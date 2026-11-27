// -----------------------------------------------------------------------------
// i2c_qch_pkg - Arm LPI Q-Channel agent: model of the clock controller that gates PCLK of apb_i2c
//               (spec 3.8, 4.12, 6.17; Arm IHI 0068 Q-Channel)
//
//   i2c_qch_item  one handshake: QCH_EV_REQ (Q_RUN -> Q_REQUEST -> Q_STOPPED | Q_DENIED -> Q_CONTINUE -> Q_RUN)
//                 or QCH_EV_EXIT (Q_STOPPED -> Q_EXIT -> Q_RUN) with the wake source
//   i2c_qch_cfg   QREQn reset level, automatic exit after reset, automatic wake-up on QACTIVE (latency range),
//                 clock gating on/off, response timeout
//   i2c_qch_ctrl  active controller:
//                 - request()  : quiescence request; on accept the PCLK gate is closed (cfg.gate)
//                 - exit()     : controller-initiated exit (clock restarted, QREQn = 1)
//                 - auto wake  : while stopped, QACTIVE is synchronized (2 FF on the free-running clock); once high
//                                the controller waits wake_lat_min..max cycles, restarts the clock and exits.
//                                The wake source (APB select / bus line low / DMAC end request / DMA request /
//                                other) is classified from the TB observation signals at the first QACTIVE sample.
//                 - checks     : response within resp_tmo cycles, QACCEPTn = 1 after Q_CONTINUE, accept only
//                                while the IP is idle (no bus busy / controller / target active / DMA request,
//                                spec 4.12) when chk_accept_idle = 1
//                 - PRESETn    : QREQn returns to its reset level, the clock gate opens; with qreqn_rst = 0 and
//                                exit_after_reset = 1 the controller exits Q_STOPPED after every reset
// -----------------------------------------------------------------------------
`ifndef I2C_QCH_PKG_SV
`define I2C_QCH_PKG_SV

`include "uvm_macros.svh"

package i2c_qch_pkg;
  import uvm_pkg::*;

  typedef virtual i2c_ip_qch_if qch_vif_t;

  typedef enum bit [2:0] { Q_RUN, Q_REQUEST, Q_STOPPED, Q_EXIT, Q_DENIED, Q_CONTINUE } qch_state_e;
  typedef enum bit       { QCH_ACCEPT, QCH_DENY } qch_resp_e;
  typedef enum bit [2:0] { QW_CTRL, QW_APB, QW_BUS, QW_DMA_END, QW_DMA_REQ, QW_OTHER } qch_wake_e;
  typedef enum bit       { QCH_EV_REQ, QCH_EV_EXIT } qch_ev_e;

  // ---------------------------------------------------------------- item
  class i2c_qch_item extends uvm_sequence_item;
    `uvm_object_utils(i2c_qch_item)
    qch_ev_e     kind = QCH_EV_REQ;
    qch_resp_e   resp;
    qch_wake_e   wake;
    bit          qactive_at_req;      // QACTIVE (sampled) when QREQn was lowered
    bit [3:0]    ctx;                 // {dma_req, tgt_active, ctrl_active, bus_busy} at the response
    int unsigned resp_cyc;            // QREQn 1->0 to QACCEPTn 1->0 / QDENY 0->1
    int unsigned exit_cyc;            // QREQn 0->1 to QACCEPTn 0->1
    int unsigned stop_cyc;            // cycles in Q_STOPPED
    int unsigned wake_lat;            // extra controller latency of an automatic wake-up
    function new(string name = "i2c_qch_item"); super.new(name); endfunction
    virtual function string convert2string();
      if (kind == QCH_EV_REQ)
        return $sformatf("REQ %s after %0d cyc (QACTIVE=%0d ctx=%b)", resp.name(), resp_cyc, qactive_at_req, ctx);
      return $sformatf("EXIT %s after %0d stopped cyc, lat %0d, QACCEPTn after %0d cyc", wake.name(), stop_cyc,
                       wake_lat, exit_cyc);
    endfunction
  endclass : i2c_qch_item

  // ---------------------------------------------------------------- configuration
  class i2c_qch_cfg extends uvm_object;
    `uvm_object_utils(i2c_qch_cfg)
    qch_vif_t    vif;
    bit          qreqn_rst        = 1'b1;   // QREQn level during PRESETn (= DUT parameter QREQN_RST)
    bit          exit_after_reset = 1'b1;   // qreqn_rst = 0: leave Q_STOPPED right after PRESETn
    bit          auto_wake        = 1'b1;   // exit Q_STOPPED when QACTIVE rises
    int unsigned wake_lat_min     = 0;
    int unsigned wake_lat_max     = 4;
    bit          gate             = 1'b1;   // stop the DUT PCLK in Q_STOPPED
    int unsigned resp_tmo         = 32;     // cycles
    bit          chk_accept_idle  = 1'b1;
    function new(string name = "i2c_qch_cfg"); super.new(name); endfunction
  endclass : i2c_qch_cfg

  // ---------------------------------------------------------------- clock controller model
  class i2c_qch_ctrl extends uvm_component;
    `uvm_component_utils(i2c_qch_ctrl)

    i2c_qch_cfg  cfg;
    qch_vif_t    vif;
    uvm_analysis_port #(i2c_qch_item) ap;

    qch_state_e  state = Q_RUN;
    int unsigned n_accept, n_deny, n_exit;
    int unsigned n_wake[qch_wake_e];
    longint unsigned cyc;                    // free-running clock cycles

    protected semaphore  lock;
    protected bit        qa_s1, qa_s2;       // QACTIVE synchronizer
    protected bit        wake_seen;
    protected qch_wake_e wake_cause;
    protected longint unsigned stop_t0;

    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (cfg == null && !uvm_config_db #(i2c_qch_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("QCH_CFG", "no i2c_qch_cfg")
      vif  = cfg.vif;
      lock = new(1);
    endfunction

    // ============================================================ API (called from sequences)
    // quiescence request; returns the response of the IP. Must be called in Q_RUN.
    task request(output qch_resp_e r, input bit gate_clk = 1'b1);
      i2c_qch_item it = i2c_qch_item::type_id::create("req");
      int unsigned n = 0;
      r = QCH_DENY;
      lock.get();
      if (state != Q_RUN) begin
        `uvm_error("QCH_STATE", $sformatf("request() in %s", state.name()))
        lock.put();
        return;
      end
      @(vif.drv_cb);
      it.qactive_at_req = (vif.drv_cb.qactive === 1'b1);
      vif.drv_cb.qreqn <= 1'b0;
      state = Q_REQUEST;
      do begin @(vif.drv_cb); n++; end
      while ((vif.drv_cb.qacceptn === 1'b1) && (vif.drv_cb.qdeny === 1'b0) && (n < cfg.resp_tmo) && (vif.rst_n === 1'b1));
      it.resp_cyc = n;
      it.ctx = {vif.drv_cb.o_dma_req === 1'b1, vif.drv_cb.o_tgt_active === 1'b1,
                vif.drv_cb.o_ctrl_active === 1'b1, vif.drv_cb.o_bus_busy === 1'b1};
      if (vif.rst_n !== 1'b1) begin lock.put(); return; end
      if ((vif.drv_cb.qacceptn === 1'b0) && (vif.drv_cb.qdeny === 1'b0)) begin
        r = QCH_ACCEPT;
        it.resp = QCH_ACCEPT;
        n_accept++;
        state     = Q_STOPPED;
        wake_seen = 1'b0;
        stop_t0   = cyc;
        if (cfg.chk_accept_idle && (it.ctx != 0))
          `uvm_error("QCH_ACCEPT_BUSY", $sformatf("Q-Channel request accepted while the IP is active: %s", it.convert2string()))
        if (gate_clk && cfg.gate) vif.drv_cb.cg_en <= 1'b0;
      end else if ((vif.drv_cb.qacceptn === 1'b1) && (vif.drv_cb.qdeny === 1'b1)) begin
        r = QCH_DENY;
        it.resp = QCH_DENY;
        n_deny++;
        state = Q_DENIED;
        vif.drv_cb.qreqn <= 1'b1;
        state = Q_CONTINUE;
        n = 0;
        do begin @(vif.drv_cb); n++; end
        while ((vif.drv_cb.qdeny !== 1'b0) && (n < cfg.resp_tmo) && (vif.rst_n === 1'b1));
        if (vif.drv_cb.qdeny !== 1'b0) `uvm_error("QCH_CONTINUE", "QDENY not cleared after QREQn = 1 (Q_CONTINUE)")
        if (vif.drv_cb.qacceptn !== 1'b1) `uvm_error("QCH_CONTINUE", "QACCEPTn changed in Q_CONTINUE")
        state = Q_RUN;
      end else begin
        `uvm_error("QCH_RESP", $sformatf("no response to QREQn = 0 within %0d cycles (QACCEPTn=%b QDENY=%b)",
                                         cfg.resp_tmo, vif.drv_cb.qacceptn, vif.drv_cb.qdeny))
        vif.drv_cb.qreqn <= 1'b1;
        state = Q_RUN;
      end
      `uvm_info("QCH", it.convert2string(), UVM_MEDIUM)
      ap.write(it);
      lock.put();
    endtask

    // controller-initiated exit from Q_STOPPED (no-op in any other state)
    task exit();
      lock.get();
      if (state == Q_STOPPED) do_exit(QW_CTRL, 0);
      lock.put();
    endtask

    // wait until the channel is in Q_RUN (e.g. after an automatic wake-up)
    task wait_run(realtime tmo = 100us);
      bit to = 0;
      if (state == Q_RUN) return;
      fork begin
        fork
          wait (state == Q_RUN);
          begin #(tmo); to = 1; end
        join_any
        disable fork;
      end join
      if (to) `uvm_error("QCH_TMO", $sformatf("Q-Channel not back in Q_RUN within %0t (state %s)", tmo, state.name()))
    endtask

    function bit stopped(); return state == Q_STOPPED; endfunction

    // ============================================================ internals
    protected task do_exit(qch_wake_e w, int unsigned lat);
      i2c_qch_item it = i2c_qch_item::type_id::create("exit");
      int unsigned n = 0;
      it.kind     = QCH_EV_EXIT;
      it.wake     = w;
      it.wake_lat = lat;
      it.stop_cyc = int'(cyc - stop_t0);
      @(vif.drv_cb);
      vif.drv_cb.cg_en <= 1'b1;
      vif.drv_cb.qreqn <= 1'b1;
      state = Q_EXIT;
      do begin @(vif.drv_cb); n++; end
      while ((vif.drv_cb.qacceptn !== 1'b1) && (n < cfg.resp_tmo) && (vif.rst_n === 1'b1));
      if (vif.rst_n !== 1'b1) return;
      if (vif.drv_cb.qacceptn !== 1'b1) `uvm_error("QCH_EXIT", $sformatf("QACCEPTn not raised within %0d cycles of QREQn = 1", cfg.resp_tmo))
      if (vif.drv_cb.qdeny !== 1'b0)    `uvm_error("QCH_EXIT", "QDENY = 1 in Q_EXIT")
      it.exit_cyc = n;
      n_exit++;
      if (!n_wake.exists(w)) n_wake[w] = 0;
      n_wake[w]++;
      state = Q_RUN;
      `uvm_info("QCH", it.convert2string(), UVM_MEDIUM)
      ap.write(it);
    endtask

    protected function qch_wake_e classify();
      if (vif.drv_cb.o_psel === 1'b1)                                    return QW_APB;
      if (vif.drv_cb.o_end_req === 1'b1)                                 return QW_DMA_END;
      if ((vif.drv_cb.o_scl === 1'b0) || (vif.drv_cb.o_sda === 1'b0))    return QW_BUS;
      if (vif.drv_cb.o_dma_req === 1'b1)                                 return QW_DMA_REQ;
      return QW_OTHER;
    endfunction

    protected task sync_loop();
      forever begin
        @(vif.drv_cb);
        cyc++;
        qa_s2 = qa_s1;
        qa_s1 = (vif.drv_cb.qactive === 1'b1);
        if ((state == Q_STOPPED) && !wake_seen && (vif.drv_cb.qactive === 1'b1)) begin
          wake_seen  = 1'b1;
          wake_cause = classify();
        end
      end
    endtask

    protected task wake_loop();
      if ((state == Q_STOPPED) && cfg.exit_after_reset) exit();
      forever begin
        @(vif.drv_cb);
        if (cfg.auto_wake && (state == Q_STOPPED) && qa_s2) begin
          int unsigned lat = $urandom_range(cfg.wake_lat_max, cfg.wake_lat_min);
          repeat (lat) @(vif.drv_cb);
          lock.get();
          if (state == Q_STOPPED) do_exit(wake_seen ? wake_cause : QW_OTHER, lat);
          lock.put();
        end
      end
    endtask

    task run_phase(uvm_phase phase);
      fork sync_loop(); join_none
      forever begin
        // PRESETn: the controller is reset too
        lock      = new(1);
        vif.qreqn = cfg.qreqn_rst;
        vif.cg_en = 1'b1;
        state     = cfg.qreqn_rst ? Q_RUN : Q_STOPPED;
        wake_seen = 1'b0;
        stop_t0   = cyc;
        wait (vif.rst_n === 1'b1);
        fork begin
          fork
            wake_loop();
            @(negedge vif.rst_n);
          join_any
          disable fork;
        end join
      end
    endtask

    function void report_phase(uvm_phase phase);
      string s = "";
      foreach (n_wake[w]) s = {s, $sformatf(" %s=%0d", w.name(), n_wake[w])};
      `uvm_info("QCH", $sformatf("Q-Channel: %0d accepted, %0d denied, %0d exits (wake:%s)", n_accept, n_deny, n_exit, s), UVM_LOW)
    endfunction
  endclass : i2c_qch_ctrl
endpackage : i2c_qch_pkg

`endif
