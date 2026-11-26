// -----------------------------------------------------------------------------
// i2c_dma_pkg - DMAC agent for the apb_i2c DMA channel (spec 3.4, 4.9, 6.15)
//
//   i2c_dma_item     one observed event: request handshake (req_type, ack_type), request withdrawn without ack
//                    (DMA_CTRL.CLR), late ack seen with req = 0, end handshake (4-phase or pulse)
//   i2c_dma_op       one DMA operation programmed by a test (FIFO, flow control, block size, buffer)
//   i2c_dmac_driver  reactive DMAC: waits for dma_req, performs the APB element transfers (fixed peripheral address,
//                    32-bit) through the APB VIP sequencer, answers with ack / ack_type, holds ack until req = 0
//                    (spec 4.9.3 rules 2-3). Without a running operation (channel stopped) every request is DENYed
//                    ("request clear", spec 4.9.5) - or, with cfg.no_deny (DMAC without DENY), left pending until a
//                    new operation starts or the IP withdraws it (DMA_CTRL.CLR, spec 4.9.7).
//                    When an operation ends the DMAC signals it on dma_end_req (spec 4.9.6): cfg.end_mode = END_HS
//                    (4-phase, like a DMA-350 trigger output), END_PULSE (1-cycle terminal-count pulse) or END_NONE.
//   i2c_dma_monitor  passive: publishes every observed event
//   i2c_dma_agent    cfg.active = 0 -> monitor only
// -----------------------------------------------------------------------------
`ifndef I2C_DMA_PKG_SV
`define I2C_DMA_PKG_SV

`include "uvm_macros.svh"

package i2c_dma_pkg;
  import uvm_pkg::*;
  import apb_pkg::*;

  typedef virtual i2c_ip_dma_if dma_vif_t;

  typedef enum bit [1:0] { DMA_SINGLE = 2'b00, DMA_LAST_SINGLE = 2'b01, DMA_BLOCK = 2'b10, DMA_LAST_BLOCK = 2'b11 } dma_req_e;
  typedef enum bit [1:0] { DMA_OKAY = 2'b00, DMA_DENY = 2'b01, DMA_LAST_OKAY = 2'b10, DMA_ACK_RSVD = 2'b11 } dma_ack_e;
  typedef enum bit [1:0] { SEL_CMD = 2'd0, SEL_CRX = 2'd1, SEL_TTX = 2'd2, SEL_TRX = 2'd3 } dma_sel_e;
  typedef enum bit [1:0] { END_NONE = 2'd0, END_HS = 2'd1, END_PULSE = 2'd2 } dma_end_mode_e;
  typedef enum bit [2:0] { DMA_EV_HS, DMA_EV_WITHDRAW, DMA_EV_LATE_ACK, DMA_EV_END_HS, DMA_EV_END_PULSE } dma_ev_e;

  // data register of every FIFO (spec 3.4 / 4.9.5)
  function automatic bit [11:0] dma_sel_addr(dma_sel_e s);
    case (s)
      SEL_CMD: return 12'h01C;
      SEL_CRX: return 12'h020;
      SEL_TTX: return 12'h024;
      default: return 12'h028;
    endcase
  endfunction

  function automatic bit dma_sel_is_rx(dma_sel_e s);
    return (s == SEL_CRX) || (s == SEL_TRX);
  endfunction

  // ---------------------------------------------------------------- item
  class i2c_dma_item extends uvm_sequence_item;
    `uvm_object_utils(i2c_dma_item)
    dma_ev_e     kind = DMA_EV_HS;
    dma_req_e    req_type;
    dma_ack_e    ack_type;
    bit          req_pending;           // END: dma_req_o was 1 when the end event was sampled
    int unsigned n_elems;
    time         t_req, t_ack;
    function new(string name = "i2c_dma_item"); super.new(name); endfunction
    virtual function string convert2string();
      if (kind != DMA_EV_HS) return $sformatf("%s req_pending=%0d", kind.name(), req_pending);
      return $sformatf("%s(%s) n=%0d", req_type.name(), ack_type.name(), n_elems);
    endfunction
  endclass : i2c_dma_item

  // ---------------------------------------------------------------- cfg
  class i2c_dma_cfg extends uvm_object;
    `uvm_object_utils(i2c_dma_cfg)
    dma_vif_t    vif;
    bit          active      = 1'b1;
    int unsigned lat_min     = 2;      // cycles between seeing req and the first APB access ("DMAC arbitration")
    int unsigned lat_max     = 6;
    int unsigned ack_min     = 0;      // extra cycles before ack after the last element
    int unsigned ack_max     = 2;
    dma_end_mode_e end_mode  = END_HS; // end handshake after every finished operation (DMA-350 trigger output)
    int unsigned end_lat_min = 0;      // cycles from the final ack handshake to dma_end_req
    int unsigned end_lat_max = 3;
    int unsigned end_hold_max = 2;     // END_HS: extra cycles end_req stays 1 after end_ack
    bit          no_deny     = 0;      // DMAC without DENY: requests of a stopped channel stay pending
    bit          ack_after_withdraw = 0;   // serve (and late-ack) a request withdrawn while the DMAC arbitrated
    int unsigned late_ack_hold = 0;    // extra cycles a late ack (req already 0) stays 1
    function new(string name = "i2c_dma_cfg"); super.new(name); endfunction
  endclass : i2c_dma_cfg

  // ---------------------------------------------------------------- one DMA operation (programmed DMAC channel)
  class i2c_dma_op extends uvm_object;
    `uvm_object_utils(i2c_dma_op)
    dma_sel_e      sel;
    bit            pfc;                 // 1: operation ends on LAST_SINGLE / LAST_BLOCK (peripheral flow control)
    int unsigned   count;               // DMAC flow control: total elements; PFC: buffer size (upper bound)
    int unsigned   blk;                 // DMAC block size (= DMA_BLK.BLK)
    bit            deny_single;         // DENY SINGLE while >= blk elements remain (spec 4.9.4 example)
    bit            okay_as_rsvd;        // answer OKAY with the reserved code 2'b11 (IP must treat it as OKAY)
    bit            no_last_okay;        // DMAC without LAST_OKAY: the final ack is OKAY, the end is signalled on
                                        // dma_end_req only (spec 4.9.6, DMA_CTRL.END_EN = 1)
    bit [31:0]     tx_data[$];          // TX FIFO (CMD/TTX): words written to the IP
    bit [31:0]     rx_data[$];          // RX FIFO (CRX/TRX): words read from the IP
    string         log;                 // "BLOCK SINGLE(D) ..." sequence of requests seen
    dma_req_e      req_log[$];
    dma_ack_e      ack_log[$];
    int unsigned   moved;               // elements transferred
    bit            done;                // operation finished (LAST_OKAY sent or LAST_* served)
    bit            stop;                // stop the channel: pending / future requests are DENYed
    int unsigned   n_slverr;
    uvm_event      done_ev;
    function new(string name = "i2c_dma_op");
      super.new(name);
      done_ev = new("done_ev");
    endfunction
    function int unsigned remaining();
      return (count > moved) ? count - moved : 0;
    endfunction
  endclass : i2c_dma_op

  // ---------------------------------------------------------------- DMAC driver (reactive responder)
  class i2c_dmac_driver extends uvm_component;
    `uvm_component_utils(i2c_dmac_driver)

    i2c_dma_cfg   cfg;
    apb_sequencer apb_sqr;              // set by the env: the DMAC is a second APB requester behind the same bus
    i2c_dma_op    op;                   // current operation (null = channel stopped)
    apb_api_seq   api;
    int unsigned  n_req, n_deny, n_end, n_withdrawn, n_late_ack;
    semaphore     end_sem = new(1);

    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    // program / start an operation; the previous one is dropped
    function void start_op(i2c_dma_op o);
      op = o;
      `uvm_info("DMAC", $sformatf("operation started: sel=%s pfc=%0d count=%0d blk=%0d deny_single=%0d",
                o.sel.name(), o.pfc, o.count, o.blk, o.deny_single), UVM_MEDIUM)
    endfunction

    // stop the channel: every pending / new request is answered with DENY
    function void stop_op();
      if (op != null) op.stop = 1;
      op = null;
    endfunction

    task run_phase(uvm_phase phase);
      cfg.vif.ack      <= 1'b0;
      cfg.vif.ack_type <= 2'b00;
      cfg.vif.end_req  <= 1'b0;
      api = apb_api_seq::type_id::create("dmac_api");
      fork api.start(apb_sqr); join_none
      forever begin
        wait (cfg.vif.rst_n === 1'b1);
        fork
          serve_loop();
          @(negedge cfg.vif.rst_n);
        join_any
        disable fork;
        cfg.vif.ack      <= 1'b0;
        cfg.vif.ack_type <= 2'b00;
        cfg.vif.end_req  <= 1'b0;
        end_sem = new(1);
      end
    endtask

    // end of a DMA operation on dma_end_req (spec 4.9.6): pulse = 1-cycle terminal-count pulse, else 4-phase
    task send_end(bit pulse, int unsigned lat = 0);
      int unsigned k;
      end_sem.get();
      repeat (lat) @(cfg.vif.drv_cb);
      cfg.vif.drv_cb.end_req <= 1'b1;
      if (pulse) begin
        @(cfg.vif.drv_cb);
        cfg.vif.drv_cb.end_req <= 1'b0;
        @(cfg.vif.drv_cb);
      end else begin
        k = 0;
        do begin @(cfg.vif.drv_cb); k++; end while (cfg.vif.drv_cb.end_ack !== 1'b1 && k < 64);
        if (k >= 64) `uvm_error("DMAC", "no dma_end_ack_o for dma_end_req_i")
        repeat ($urandom_range(cfg.end_hold_max, 0)) @(cfg.vif.drv_cb);
        cfg.vif.drv_cb.end_req <= 1'b0;
        k = 0;
        do begin @(cfg.vif.drv_cb); k++; end while (cfg.vif.drv_cb.end_ack !== 1'b0 && k < 64);
        if (k >= 64) `uvm_error("DMAC", "dma_end_ack_o stuck at 1 after dma_end_req_i = 0")
      end
      n_end++;
      end_sem.put();
    endtask

    protected task serve_loop();
      forever begin
        dma_req_e  rt;
        dma_ack_e  at;
        int unsigned n;
        bit        late;
        i2c_dma_op o;
        do @(cfg.vif.drv_cb); while (cfg.vif.drv_cb.req !== 1'b1);
        rt = dma_req_e'(cfg.vif.drv_cb.req_type);
        n_req++;
        repeat ($urandom_range(cfg.lat_max, cfg.lat_min)) @(cfg.vif.drv_cb);
        // a request withdrawn (DMA_CTRL.CLR) before the DMAC started to serve it is not served
        if (cfg.vif.drv_cb.req !== 1'b1 && !cfg.ack_after_withdraw) begin n_withdrawn++; continue; end
        o = op;
        if (cfg.no_deny) begin
          // DMAC without DENY: the request of a stopped channel stays pending (new operation or withdrawal)
          while ((o == null || o.stop || o.done) && cfg.vif.drv_cb.req === 1'b1) begin @(cfg.vif.drv_cb); o = op; end
          if (cfg.vif.drv_cb.req !== 1'b1 && !cfg.ack_after_withdraw) begin n_withdrawn++; continue; end
        end
        late = (cfg.vif.drv_cb.req !== 1'b1);
        if (late) n_late_ack++;
        n = 0;
        if (o == null || o.stop || o.done) begin
          at = DMA_DENY;
        end else if (o.deny_single && rt == DMA_SINGLE && o.blk != 0 && o.remaining() >= o.blk) begin
          at = DMA_DENY;
        end else begin
          n = (rt == DMA_BLOCK || rt == DMA_LAST_BLOCK) ? o.blk : 1;
          if (!o.pfc && n > o.remaining()) n = o.remaining();      // final (short) block of a DMAC-controlled operation
          if (o.pfc && n > o.remaining())
            `uvm_error("DMAC", $sformatf("%s request of %0d elements exceeds the remaining buffer (%0d)", rt.name(), n, o.remaining()))
          xfer(o, n);
          if (o.pfc) begin
            at = DMA_OKAY;
            if (rt == DMA_LAST_SINGLE || rt == DMA_LAST_BLOCK) o.done = 1;
          end else if (o.remaining() == 0) begin
            at = o.no_last_okay ? DMA_OKAY : DMA_LAST_OKAY;
            o.done = 1;
          end else begin
            at = DMA_OKAY;
          end
          if (at == DMA_OKAY && o.okay_as_rsvd) at = DMA_ACK_RSVD;
        end
        if (at == DMA_DENY) n_deny++;
        if (o != null) begin
          o.req_log.push_back(rt); o.ack_log.push_back(at);
          o.log = {o.log, (o.log == "") ? "" : " ", rt.name(), (at == DMA_DENY) ? "(D)" : ""};
        end
        repeat ($urandom_range(cfg.ack_max, cfg.ack_min)) @(cfg.vif.drv_cb);
        cfg.vif.drv_cb.ack      <= 1'b1;
        cfg.vif.drv_cb.ack_type <= at;
        do @(cfg.vif.drv_cb); while (cfg.vif.drv_cb.req !== 1'b0);      // hold ack until req = 0 (rule 3)
        if (late) repeat (cfg.late_ack_hold) @(cfg.vif.drv_cb);
        cfg.vif.drv_cb.ack      <= 1'b0;
        cfg.vif.drv_cb.ack_type <= 2'b00;
        @(cfg.vif.drv_cb);
        if (o != null && o.done && !o.done_ev.is_on()) begin
          `uvm_info("DMAC", $sformatf("operation done: %s (%0d elements)", o.log, o.moved), UVM_MEDIUM)
          if (op == o) op = null;
          if (cfg.end_mode != END_NONE) send_end(cfg.end_mode == END_PULSE, $urandom_range(cfg.end_lat_max, cfg.end_lat_min));
          o.done_ev.trigger();
        end
      end
    endtask

    // the element transfers: one 32-bit APB access each, fixed peripheral address
    protected task xfer(i2c_dma_op o, int unsigned n);
      bit [11:0] a = dma_sel_addr(o.sel);
      repeat (n) begin
        bit err;
        bit [31:0] d;
        if (dma_sel_is_rx(o.sel)) begin
          api.read(a, d, err);
          o.rx_data.push_back(d);
        end else begin
          d = (o.moved < o.tx_data.size()) ? o.tx_data[o.moved] : 32'h0;
          api.write(a, d, err);
        end
        if (err) begin
          o.n_slverr++;
          `uvm_error("DMAC", $sformatf("PSLVERR on DMA element %0d (addr 0x%03h)", o.moved, a))
        end
        o.moved++;
      end
    endtask
  endclass : i2c_dmac_driver

  // ---------------------------------------------------------------- monitor
  class i2c_dma_monitor extends uvm_monitor;
    `uvm_component_utils(i2c_dma_monitor)
    i2c_dma_cfg cfg;
    uvm_analysis_port #(i2c_dma_item) ap;
    int unsigned n_hs;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase); super.build_phase(phase); ap = new("ap", this); endfunction
    int unsigned n_withdraw, n_late, n_end;

    function void publish(dma_ev_e k, bit pend = 0);
      i2c_dma_item e = i2c_dma_item::type_id::create("dma_ev");
      e.kind = k; e.req_pending = pend; e.t_req = $time;
      `uvm_info("DMA_MON", e.convert2string(), UVM_HIGH)
      ap.write(e);
    endfunction

    task run_phase(uvm_phase phase);
      i2c_dma_item it;
      bit req_p = 0, ack_p = 0, end_p = 0, end_acked = 0, end_pend = 0;
      forever begin
        @(cfg.vif.mon_cb);
        if (cfg.vif.rst_n !== 1'b1) begin it = null; req_p = 0; ack_p = 0; end_p = 0; continue; end
        // request handshake
        if (cfg.vif.mon_cb.req === 1'b1 && !req_p) begin
          it = i2c_dma_item::type_id::create("dma_hs");
          it.req_type = dma_req_e'(cfg.vif.mon_cb.req_type);
          it.t_req = $time;
        end
        if (it != null && cfg.vif.mon_cb.req === 1'b1 && cfg.vif.mon_cb.ack === 1'b1) begin
          it.ack_type = dma_ack_e'(cfg.vif.mon_cb.ack_type);
          it.t_ack = $time;
          n_hs++;
          `uvm_info("DMA_MON", it.convert2string(), UVM_HIGH)
          ap.write(it);
          it = null;
        end
        if (it != null && cfg.vif.mon_cb.req !== 1'b1 && req_p) begin      // dropped without ack: DMA_CTRL.CLR
          n_withdraw++;
          publish(DMA_EV_WITHDRAW);
          it = null;
        end
        if (cfg.vif.mon_cb.ack === 1'b1 && !ack_p && cfg.vif.mon_cb.req !== 1'b1) begin
          n_late++;
          publish(DMA_EV_LATE_ACK);
        end
        // end handshake: published when dma_end_req falls (4-phase if end_ack was seen while end_req = 1)
        if (cfg.vif.mon_cb.end_req === 1'b1 && !end_p) begin end_acked = 0; end_pend = (cfg.vif.mon_cb.req === 1'b1); end
        if (cfg.vif.mon_cb.end_req === 1'b1 && cfg.vif.mon_cb.end_ack === 1'b1) end_acked = 1;
        if (cfg.vif.mon_cb.end_req !== 1'b1 && end_p) begin
          n_end++;
          publish(end_acked ? DMA_EV_END_HS : DMA_EV_END_PULSE, end_pend);
        end
        req_p = (cfg.vif.mon_cb.req === 1'b1);
        ack_p = (cfg.vif.mon_cb.ack === 1'b1);
        end_p = (cfg.vif.mon_cb.end_req === 1'b1);
      end
    endtask
  endclass : i2c_dma_monitor

  // ---------------------------------------------------------------- agent
  class i2c_dma_agent extends uvm_agent;
    `uvm_component_utils(i2c_dma_agent)
    i2c_dma_cfg     cfg;
    i2c_dmac_driver drv;
    i2c_dma_monitor mon;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(i2c_dma_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("DMA_CFG", "no i2c_dma_cfg")
      mon = i2c_dma_monitor::type_id::create("mon", this);
      if (cfg.active) drv = i2c_dmac_driver::type_id::create("drv", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      mon.cfg = cfg;
      if (drv != null) drv.cfg = cfg;
    endfunction
  endclass : i2c_dma_agent

endpackage : i2c_dma_pkg

`endif
