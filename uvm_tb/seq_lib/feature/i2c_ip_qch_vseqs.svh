// -----------------------------------------------------------------------------
// Layer 2 - Q-Channel feature sequences (VP-QCH, spec 3.8, 4.12, 6.17)
//   i2c_qch_handshake_vseq  VP-QCH-01  idle IP: accept -> Q_STOPPED (QACTIVE low, no spontaneous wake-up) ->
//                                      controller exit, with CTRL.EN = 0 / 1, clock gated / kept running; CTRL.EN = 0:
//                                      bus traffic does not wake the IP
//   i2c_qch_deny_vseq       VP-QCH-02  deny while active: controller transfer / hold, target addressed, bus busy
//                                      (other devices), bus-idle detection after enable, SCL pulled low while stopped
//                                      (wake-up) and held low, DMA request
//                                      pending with CTRL.EN = 0; accepted again once idle
//   i2c_qch_apb_wake_vseq   VP-QCH-03  APB access while stopped: QACTIVE (PSEL) wake-up, PREADY = 0 until Q_RUN,
//                                      exactly one register / FIFO side effect, PSLVERR rules unchanged
//   i2c_qch_bus_wake_vseq   VP-QCH-04  bus activity while stopped (CTRL.EN = 1): wake-up within tHD;STA, target
//                                      write / read / other address served correctly
//   i2c_qch_ctrl_vseq       VP-QCH-05  controller transfers started from Q_STOPPED (CMD write wakes the IP)
//   i2c_qch_dma_vseq        VP-QCH-06  DMAC end request wakes the IP; TRX DMA (PFC) with the clock stopped between
//                                      transactions while DMA_CTRL.EN = 1
//   i2c_qch_reset_vseq      VP-QCH-07  reset state (QREQN_RST), first access from Q_STOPPED, PRESETn while stopped
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- VP-QCH-01
class i2c_qch_handshake_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_handshake_vseq)

  rand int unsigned n_cycles = 6;
  constraint c_def { soft n_cycles inside {[4:10]}; }

  function new(string name = "i2c_qch_handshake_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v;
    bit        e;
    int unsigned cyc;
    int unsigned ref_cyc;
    #(200ns);                                                 // last APB access of the initialisation finished (PSEL = 0)
    chk(qch_pins() == 3'b100, $sformatf("idle IP in Q_RUN: {QACCEPTn,QDENY,QACTIVE} = %b, expected 100", qch_pins()));
    timed_xfer(0, R_VERSION, v, e, ref_cyc);
    for (int i = 0; i < n_cycles; i++) begin
      i2c_ip_qch_cycle_vseq c = i2c_ip_qch_cycle_vseq::type_id::create($sformatf("cycle%0d", i));
      bit en = $urandom_range(1);
      if (en) begin enable(1, 1); wait_dut_idle(); end
      else    wr(R_CTRL, 0);
      #(2us);
      if (!c.randomize() with { gate_clk == (i != 1); }) `uvm_fatal("QCH", "randomize cycle")
      run_seq(c);
      // first access after the exit: zero wait state again
      timed_xfer(0, R_VERSION, v, e, cyc);
      chk(v == cfg.p.version && !e, $sformatf("VERSION after exit = 0x%08h", v));
      chk(cyc <= ref_cyc, $sformatf("APB access in Q_RUN takes %0d cycles (%0d before any stop)", cyc, ref_cyc));
    end
    // CTRL.EN = 0: bus activity between other devices is not a wake source (4.12.3), the IP stays stopped
    begin
      bit [7:0] d[$];
      bit       ok;
      wr(R_CTRL, 0);
      #(2us);
      qch_stop("CTRL.EN = 0, before external traffic");
      rnd_bytes(3, d);
      d.push_front(8'h70);
      ext_write(7'h50, d, ok);
      chk(ok, "external write between other devices");
      chk(qch_stopped() && qch_pins() == 3'b000,
          $sformatf("CTRL.EN = 0: still Q_STOPPED, QACTIVE low after bus traffic ({QACCEPTn,QDENY,QACTIVE} = %b)", qch_pins()));
      qch_exit();
    end
    enable(1, 1);
    wait_dut_idle();
  endtask
endclass : i2c_qch_handshake_vseq

// ---------------------------------------------------------------- VP-QCH-02
class i2c_qch_deny_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_deny_vseq)
  function new(string name = "i2c_qch_deny_vseq"); super.new(name); endfunction

  task accept_again(string what);
    qch_stop(what);
    qch_exit();
  endtask

  task main();
    bit [7:0]  d[$];
    bit [9:0]  q[$], exp[$];
    bit [31:0] v;
    bit        ok;
    int unsigned nb;

    // controller transfer (owns the bus, holds SCL while the CMD FIFO refills)
    rnd_bytes(10, d);
    d.push_front(8'h40);
    fork
      ctrl_write(7'h50, d);
      begin
        wait (side.d_ctrl_active === 1'b1);
        #(3us);
        qch_expect(QCH_DENY, "controller transfer in progress:");
      end
    join
    accept_again("after the controller transfer");

    // controller holding SCL: data byte without STOP, CMD FIFO empty
    push_cmds('{c_start(7'h50, 0), c_wr(8'h60)});
    wait (side.d_ctrl_hold === 1'b1);
    #(2us);
    qch_expect(QCH_DENY, "controller holds SCL (CMD empty):");
    push_cmds('{c_wr(8'h5A, 1)});
    wait_cmpl(0, "held write");
    accept_again("after the held write");

    // target addressed by the external controller
    nb = (depth > 4) ? depth - 3 : 1;
    rnd_bytes(nb, d);
    fork
      ext_write(own0, d, ok);
      begin
        wait (side.d_tgt_active === 1'b1);
        #(2us);
        qch_expect(QCH_DENY, "target addressed:");
      end
    join
    chk(ok, "external write to the DUT target ACKed");
    drain_trx(q);
    exp = '{e_start(own0, 0)};
    foreach (d[i]) exp.push_back(e_data(d[i]));
    exp.push_back(e_stop());
    chk_trx(q, exp, "target write with a denied request");
    accept_again("after the target write");

    // bus busy with a transfer between other devices
    rnd_bytes(4, d);
    d.push_front(8'h20);
    fork
      ext_write(7'h50, d, ok);
      begin
        wait (side.d_bus_busy === 1'b1);
        #(5us);
        qch_expect(QCH_DENY, "bus busy (external controller -> external target):");
      end
    join
    accept_again("after the external transfer");

    // bus-idle detection after CTRL.EN 0 -> 1 (STATUS.BUS_BUSY = 1, spec 4.3.3 / R7)
    wr(R_CTRL, 0);
    #(1us);
    accept_again("CTRL.EN = 0");
    enable(1, 1);
    qch_expect(QCH_DENY, "bus-idle detection after enable:");
    wait_dut_idle();
    accept_again("after the bus-idle detection");

    // SCL pulled low by another device while the IP is stopped: wake-up (QACTIVE from the SCL pad), then deny
    quiet();
    qch_stop("before SCL is held low");
    nb = qch_wakes(QW_BUS);
    bb(BB_HOLD_SCL);
    qch_wait_run();
    chk(qch_wakes(QW_BUS) == nb + 1, "SCL pulled low while stopped woke the IP");
    #(3us);
    qch_expect(QCH_DENY, "SCL held low:");
    bb(BB_RELEASE);
    #(30us);
    loud();
    w1c(EVENT_MASK);
    wr(R_CSTAT, 32'h7F);
    wait_dut_idle();
    accept_again("after SCL released");

    // DMA request pending (DMAC without DENY, channel not served), with the I2C core disabled (CTRL.EN = 0):
    // the DMA handshake alone keeps the IP busy (4.12.1 Q2)
    wr(R_CTRL, 0);
    cfg.dma_cfg.no_deny = 1;
    p_sequencer.dmac.stop_op();
    wr(R_DMA_BLK, 0);
    wr(R_DMA_CTRL, {26'h0, SEL_TTX, 4'b0001});
    wait (cfg.dma_cfg.vif.req === 1'b1);
    #(1us);
    qch_expect(QCH_DENY, "DMA request pending:");
    wr(R_DMA_CTRL, 32'h8);                                    // EN = 0, CLR (spec 4.9.7)
    do begin #(1us); rd(R_DMA_STAT, v); end while (v[1]);
    cfg.dma_cfg.no_deny = 0;
    wr(R_DMA_STAT, 32'h0101_0100);
    accept_again("after DMA_CTRL.CLR");
    enable(1, 1);
    wait_dut_idle();
  endtask
endclass : i2c_qch_deny_vseq

// ---------------------------------------------------------------- VP-QCH-03
class i2c_qch_apb_wake_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_apb_wake_vseq)

  rand int unsigned n_ops = 14;
  constraint c_def { soft n_ops inside {[10:20]}; }

  int unsigned ref_cyc;

  function new(string name = "i2c_qch_apb_wake_vseq"); super.new(name); endfunction

  // one access from Q_STOPPED: woken by PSEL, stretched, then zero wait state
  task stopped_xfer(bit write, bit [11:0] a, ref bit [31:0] d, output bit err, input string what);
    int unsigned cyc, w0;
    qch_stop(what);
    w0 = qch_wakes(QW_APB);
    timed_xfer(write, a, d, err, cyc);
    qch_wait_run();                                           // exit handshake ends in the same cycle as the access
    chk(qch_wakes(QW_APB) == w0 + 1, $sformatf("%s: APB access woke the IP (QACTIVE from PSEL)", what));
    chk(cyc > ref_cyc + 2, $sformatf("%s: access stretched while stopped (%0d cycles, %0d in Q_RUN)", what, cyc, ref_cyc));
    chk(!qch_stopped(), $sformatf("%s: Q_RUN after the access", what));
  endtask

  task main();
    bit [31:0] v, d;
    bit        e, ok;
    bit [9:0]  q[$];
    bit [7:0]  b[$];
    timed_xfer(0, R_VERSION, v, e, ref_cyc);
    for (int i = 0; i < n_ops; i++) begin
      int unsigned op = (i < 7) ? i : $urandom_range(6);
      case (op)
        0: begin                                             // register write + read back
          d = $urandom & 32'h00FF_FFFF;
          stopped_xfer(1, R_DEVID, d, e, "DEVID write");
          chk(!e, "DEVID write: no PSLVERR");
          stopped_xfer(0, R_DEVID, v, e, "DEVID read");
          chk(v == d && !e, $sformatf("DEVID read back 0x%08h, written 0x%08h", v, d));
        end
        1: begin                                             // CMD pushed exactly once per access
          enable(0, 1);
          d = c_start(7'h50, 0);
          stopped_xfer(1, R_CMD, d, e, "CMD push 1");
          d = c_wr(8'h11, 1);
          stopped_xfer(1, R_CMD, d, e, "CMD push 2");
          stopped_xfer(0, R_FIFO_STAT, v, e, "FIFO_STAT");
          chk(v[5:0] == 2, $sformatf("CMD level %0d after 2 pushes from Q_STOPPED", v[5:0]));
          wr(R_CTRL, 32'h0000_0105);                         // CMD_FLUSH, EN, TGT_EN
          enable(1, 1);
        end
        2: begin                                             // TTX push
          d = $urandom & 32'hFF;
          stopped_xfer(1, R_TTXDATA, d, e, "TTX push");
          rd(R_FIFO_STAT, v);
          chk(v[21:16] == 1, $sformatf("TTX level %0d after one push from Q_STOPPED", v[21:16]));
          wr(R_CTRL, 32'h0000_0407);                         // TTX_FLUSH, EN, CTRL_EN, TGT_EN
        end
        3: begin                                             // TRX pop
          bit [9:0] exp[$];
          rnd_bytes((depth > 4) ? 2 : 1, b);
          ext_write(own0, b, ok);
          stopped_xfer(0, R_TRXDATA, v, e, "TRX pop");
          chk(v[9:0] == e_start(own0, 0) && !e, $sformatf("TRXDATA = 0x%03h, expected address entry", v[9:0]));
          stopped_xfer(0, R_FIFO_STAT, v, e, "FIFO_STAT");
          chk(v[29:24] == b.size() + 1, $sformatf("TRX level %0d after one pop from Q_STOPPED", v[29:24]));
          drain_trx(q);
          foreach (b[k]) exp.push_back(e_data(b[k]));
          exp.push_back(e_stop());
          chk_trx(q, exp, "TRX rest");
        end
        4: begin                                             // CRX pop
          push_cmds('{c_start(7'h50, 1), c_rd(0, 0), c_rd(0, 1)});
          wait_cmpl(0, "read 2 bytes");
          stopped_xfer(0, R_CRXDATA, v, e, "CRX pop");
          chk(!e && v[8] == 0, $sformatf("CRXDATA first byte 0x%03h (LAST = 0)", v[8:0]));
          stopped_xfer(0, R_FIFO_STAT, v, e, "FIFO_STAT");
          chk(v[13:8] == 1, $sformatf("CRX level %0d after one pop from Q_STOPPED", v[13:8]));
          rd(R_CRXDATA, v);
          chk(v[8] == 1, "CRXDATA second byte LAST = 1");
        end
        5: begin                                             // unmapped offset: PSLVERR, read data 0 (3.7)
          stopped_xfer(0, 12'h100, v, e, "unmapped read");
          chk(e && v == 0, $sformatf("unmapped read from Q_STOPPED: PSLVERR %0d data 0x%08h", e, v));
        end
        6: begin                                             // empty CRX read: PSLVERR + FIFO_ERR (3.7)
          stopped_xfer(0, R_CRXDATA, v, e, "empty CRX read");
          chk(e && v == 0, $sformatf("empty CRXDATA read from Q_STOPPED: PSLVERR %0d data 0x%08h", e, v));
          wait_ev(I_FIFO_ERR, 100us, 1, "FIFO_ERR after the stretched read");
        end
      endcase
    end
  endtask
endclass : i2c_qch_apb_wake_vseq

// ---------------------------------------------------------------- VP-QCH-04
class i2c_qch_bus_wake_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_bus_wake_vseq)

  rand int unsigned n_msg = 6;
  constraint c_def { soft n_msg inside {[4:8]}; }

  function new(string name = "i2c_qch_bus_wake_vseq"); super.new(name); endfunction

  task main();
    // wake-up budget (spec 4.12): PCLK must run again before SCL falls after the START, i.e. within
    // tHD;STA(min) - margin (QACTIVE synchronizer of the controller, clock gate, input synchronizer)
    i2c_timing_t s = i2c_spec(cfg.i2c_mst_cfg.speed);
    int budget = int'(s.t_hd_sta / T) - 8;
    if (budget < 0) budget = 0;
    qch_auto(1, 0, budget);
    `uvm_info("QCH", $sformatf("%s: tHD;STA(min) %0.0f ns, wake-up latency 0..%0d cycles", spd.name(), s.t_hd_sta, budget), UVM_LOW)
    for (int i = 0; i < n_msg; i++) begin
      bit [7:0] d[$], r[$];
      bit [9:0] q[$], exp[$];
      bit       ok;
      bit [6:0] a = $urandom_range(1) ? own0 : own1;
      int unsigned w0, op = (i < 3) ? i : $urandom_range(2);
      if (op == 1) begin
        rnd_bytes($urandom_range(depth - 1, 1), d);
        load_ttx(d);
      end
      qch_stop("bus wake");
      w0 = qch_wakes(QW_BUS);
      case (op)
        0: begin                                             // target write
          rnd_bytes($urandom_range((depth > 4) ? depth - 3 : 1, 1), d);
          ext_write(a, d, ok);
          chk(ok, "external write to the stopped IP ACKed");
          drain_trx(q);
          exp = '{e_start(a, 0)};
          foreach (d[k]) exp.push_back(e_data(d[k]));
          exp.push_back(e_stop());
          chk_trx(q, exp, "write received after a bus wake-up");
        end
        1: begin                                             // target read (TTX loaded before the stop)
          ext_read(own0, d.size(), r, ok);
          chk(ok && r == d, $sformatf("read from the stopped IP: %p, TTX %p", r, d));
          drain_trx(q);
          w1c(EVENT_MASK);
        end
        2: begin                                             // transfer between other devices
          rnd_bytes(3, d);
          d.push_front(8'h10);
          ext_write(7'h50, d, ok);
          chk(ok, "external write to the VIP slave");
          begin
            bit [31:0] v;
            rd(R_FIFO_STAT, v);
            chk(v[29:24] == 0, "IP not addressed: TRX empty");
          end
        end
      endcase
      chk(qch_wakes(QW_BUS) == w0 + 1, $sformatf("message %0d: one wake-up by bus activity", i));
      w1c(EVENT_MASK);
    end
    qch_auto(1, 0, 4);
  endtask
endclass : i2c_qch_bus_wake_vseq

// ---------------------------------------------------------------- VP-QCH-05
class i2c_qch_ctrl_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_ctrl_vseq)

  rand int unsigned n_xfer = 6;
  constraint c_def { soft n_xfer inside {[4:8]}; }

  function new(string name = "i2c_qch_ctrl_vseq"); super.new(name); endfunction

  task main();
    for (int i = 0; i < n_xfer; i++) begin
      i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create($sformatf("cx%0d", i));
      int unsigned w0;
      if (!x.randomize() with { data.size() inside {[1:8]}; rd_len inside {[1:8]}; }) `uvm_fatal("QCH", "randomize xfer")
      qch_stop("before a controller transfer");
      w0 = qch_wakes(QW_APB);
      if (i == 0) begin
        fork
          run_seq(x);
          begin
            wait (side.d_ctrl_active === 1'b1);
            #(2us);
            qch_expect(QCH_DENY, "controller transfer started from Q_STOPPED:");
          end
        join
      end else run_seq(x);
      chk(qch_wakes(QW_APB) == w0 + 1, $sformatf("transfer %0d: the first CMD write woke the IP", i));
    end
  endtask
endclass : i2c_qch_ctrl_vseq

// ---------------------------------------------------------------- VP-QCH-06
class i2c_qch_dma_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_qch_dma_vseq)

  rand int unsigned n_trans = 3;
  constraint c_def { soft n_trans inside {[2:4]}; }

  function new(string name = "i2c_qch_dma_vseq"); super.new(name); endfunction

  task main();
    int unsigned w0;
    int unsigned blk = (depth >= 8) ? 4 : 2;
    // DMAC end request (4-phase) while the IP is stopped (spec 4.9.6): QACTIVE wake-up, DMA_STAT.END
    for (int i = 0; i < 2; i++) begin
      qch_stop("before the DMAC end request");
      w0 = qch_wakes(QW_DMA_END);
      p_sequencer.dmac.send_end(0, $urandom_range(5));
      qch_wait_run();                                         // the end handshake may finish with the exit handshake
      chk(qch_wakes(QW_DMA_END) == w0 + 1, "dma_end_req_i woke the IP");
      chk_rd(R_DMA_STAT, 32'h0100_0000, 32'h0100_0000, "DMA_STAT.END after the end handshake from Q_STOPPED");
      wr(R_DMA_STAT, 32'h0100_0000);
    end

    // target receive through DMA (SEL = TRX, PFC): the clock is stopped between transactions with DMA_CTRL.EN = 1
    for (int i = 0; i < n_trans; i++) begin
      i2c_dma_op o;
      bit [7:0] d[$];
      bit       ok;
      bit [9:0] exp[$];
      rnd_bytes($urandom_range(6, 1), d);
      dma_start(SEL_TRX, 1, 32, blk, o);
      qch_stop("DMA channel enabled, handshake idle");
      w0 = qch_wakes(QW_BUS);
      ext_write(own0, d, ok);
      dma_check(o, $sformatf("TRX DMA %0d", i), "", 2ms);
      chk(qch_wakes(QW_BUS) == w0 + 1, "woken by the START of the transaction");
      exp = '{e_start(own0, 0)};
      foreach (d[k]) exp.push_back(e_data(d[k]));
      exp.push_back(e_stop());
      chk(o.rx_data.size() == exp.size(), $sformatf("TRX DMA %0d: %0d entries moved, expected %0d", i, o.rx_data.size(), exp.size()));
      foreach (exp[k]) if (k < o.rx_data.size())
        chk(o.rx_data[k][9:0] == exp[k], $sformatf("TRX DMA %0d entry %0d = 0x%03h, expected 0x%03h", i, k, o.rx_data[k][9:0], exp[k]));
    end
  endtask
endclass : i2c_qch_dma_vseq

// ---------------------------------------------------------------- VP-QCH-07
class i2c_qch_reset_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_qch_reset_vseq)
  function new(string name = "i2c_qch_reset_vseq"); super.new(name); endfunction

  // state right after PRESETn and the first register access
  task chk_after_reset(string what);
    bit [31:0] v;
    bit        e;
    int unsigned cyc, w0;
    if (cfg.p.qreqn_rst) begin
      chk(qch_pins()[2:1] == 2'b10, $sformatf("%s: QREQN_RST = 1 -> Q_RUN (QACCEPTn = 1, QDENY = 0), got %b", what, qch_pins()[2:1]));
      timed_xfer(0, R_VERSION, v, e, cyc);
      chk(cyc <= 4, $sformatf("%s: zero wait state (%0d cycles)", what, cyc));
    end else begin
      chk(qch_stopped() && qch_pins()[2:1] == 2'b00, $sformatf("%s: QREQN_RST = 0 -> Q_STOPPED (QACCEPTn = 0), got %b", what, qch_pins()[2:1]));
      w0 = qch_wakes(QW_APB);
      timed_xfer(0, R_VERSION, v, e, cyc);
      qch_wait_run();
      chk(qch_wakes(QW_APB) == w0 + 1, $sformatf("%s: first APB access woke the IP from Q_STOPPED", what));
      chk(cyc > 6, $sformatf("%s: first access stretched until Q_RUN (%0d cycles)", what, cyc));
    end
    chk(v == cfg.p.version && !e, $sformatf("%s: VERSION = 0x%08h", what, v));
  endtask

  task main();
    i2c_ip_presetn_vseq rst_p;
    bit [7:0] d[$];
    chk_after_reset("power-on reset");
    // PRESETn while the channel is stopped (QREQN_RST = 1) or running (QREQN_RST = 0)
    init_ip(SPD_FM, 1, 1);
    if (cfg.p.qreqn_rst) qch_stop("before PRESETn");
    rst_p = i2c_ip_presetn_vseq::type_id::create("rst_p");
    rst_p.cycles = $urandom_range(1, 10);
    run_seq(rst_p);
    chk_after_reset("PRESETn pulse");
    // normal operation afterwards
    init_ip(SPD_FM, 1, 1);
    rnd_bytes(4, d);
    d.push_front(8'h30);
    ctrl_write(7'h50, d);
  endtask
endclass : i2c_qch_reset_vseq
