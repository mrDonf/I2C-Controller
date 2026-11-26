// -----------------------------------------------------------------------------
// Layer 2 - bus-level feature sequences (spec 4.2, 4.3, 4.4.6, 4.4.9, 4.4.10, 4.5.9, 4.10, 5.2.1, 6.13, 6.14)
//   The bit-bang agent is the second controller / faulty device (layer-1 i2c_ip_bb_* sequences). Around deliberate
//   protocol abuse the VIP monitor checks and the scoreboard are switched off (quiet()) and re-synchronised (loud()).
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- VP-BUS-01: arbitration lost in the address (4.4.6)
//   the winner (bit-bang) addresses the DUT target: ARB_LOST, no CMPL, the target answers; re-try (6.13)
//   requires own0 < 0x51 (the winner sends a 0 first)
class i2c_arb_lost_addr_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_addr_vseq)
  function new(string name = "i2c_arb_lost_addr_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_bb_arb_vseq bbv = i2c_ip_bb_arb_vseq::type_id::create("bbv");
    bit [7:0] wd = rnd8();
    bit [9:0] tq[$];
    if (own0 >= 7'h51) begin `uvm_error("VSEQ_CFG", "i2c_arb_lost_addr_vseq needs ADDR0 < 0x51") return; end
    bbv.bytes = '{{own0, 1'b0}, wd};
    fork
      push_cmds('{c_start(7'h51, 0), c_wr(8'hAB), c_wr(8'hCD, 1)});
      run_seq(bbv);
    join
    chk(!bbv.lost && bbv.acks[0], "winner: no loss, DUT target ACKs its address");
    chk(bbv.acks[1], "DUT target ACKs the data of the winner");
    wait_ev(I_ARB_LOST, 1ms, 1, "arbitration lost in the address");
    #(5us);
    chk_ev(I_CMPL, 0, "no CMPL after arbitration loss:");
    chk_cstat(7'h04, "arbitration lost");
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed after arbitration loss");
    chk_rd(R_STATUS, 0, 32'h2, "controller idle");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own0, 0), e_data(wd), e_stop()}, "target addressed by the winner");
    w1c(EVENT_MASK);
    ctrl_write(7'h51, '{8'hAB, 8'hCD});                      // re-try of the lost transfer (6.13)
  endtask
endclass : i2c_arb_lost_addr_vseq

// ---------------------------------------------------------------- VP-BUS-02: the other controller loses (4.4.6, 4.4.2)
class i2c_arb_win_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_win_vseq)
  function new(string name = "i2c_arb_win_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_bb_arb_vseq bbv = i2c_ip_bb_arb_vseq::type_id::create("bbv");
    bit [7:0] d[$];
    rnd_bytes(2, d);
    bbv.bytes = '{{7'h51, 1'b0}};
    fork
      ctrl_write(7'h50, '{8'h10, d[0], d[1]});
      run_seq(bbv);
    join
    chk(bbv.lost && bbv.lost_at == 0, "the bit-bang controller loses against 0x50");
    chk(slv_mem(0, 8'h10) == d[0] && slv_mem(0, 8'h11) == d[1], "transfer of the winner DUT reached the slave");
    chk_ev(I_ARB_LOST, 0, "the DUT did not lose:");
  endtask
endclass : i2c_arb_win_vseq

// ---------------------------------------------------------------- VP-BUS-03: arbitration lost in a data byte (4.4.6)
class i2c_arb_lost_data_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_data_vseq)
  function new(string name = "i2c_arb_lost_data_vseq"); super.new(name); endfunction

  rand bit [7:0] ptr = 8'h44, wd = 8'h99;              // winner pointer / data (DUT sends 0xFF: any 0 bit wins)
  constraint c_def { ptr != 8'hFF; }

  task main();
    i2c_ip_bb_arb_vseq bbv = i2c_ip_bb_arb_vseq::type_id::create("bbv");
    bbv.bytes = '{{7'h50, 1'b0}, ptr, wd};
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(8'hFF), c_wr(8'hFF, 1)});
      run_seq(bbv);
    join
    chk(!bbv.lost && bbv.acks[1], "winner writes its data byte");
    wait_ev(I_ARB_LOST, 1ms, 1, "arbitration lost in data");
    #(5us);
    chk_ev(I_CMPL, 0, "no CMPL:");
    chk_cstat(7'h04, "arbitration lost in data");
    chk(slv_mem(0, ptr) == wd, "slave got the data of the winner");
    chk_rd(R_FIFO_STAT, 0, '1, "no FIFO activity");
  endtask
endclass : i2c_arb_lost_data_vseq

// ---------------------------------------------------------------- VP-BUS-04: clock synchronisation / stretching (F6/F7)
class i2c_clk_sync_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_clk_sync_vseq)
  function new(string name = "i2c_clk_sync_vseq"); super.new(name); endfunction

  rand int unsigned n_holds = 4, hold_ns = 4000;
  constraint c_def { soft n_holds == 4; soft hold_ns == 4000; n_holds inside {[1:6]}; hold_ns inside {[200:20000]}; }

  task main();
    i2c_ip_bb_scl_stretch_vseq st = i2c_ip_bb_scl_stretch_vseq::type_id::create("st");
    bit [7:0] d[$];
    if (!st.randomize() with { n_holds == local::n_holds; hold_ns == local::hold_ns; first_rise inside {[2:9]}; rise_step inside {[3:7]}; })
      `uvm_fatal("RAND", "bb stretch")
    rnd_bytes(6, d);
    fork
      ctrl_write(7'h50, '{8'h60, d[0], d[1], d[2], d[3], d[4], d[5]});
      run_seq(st);
    join
    for (int i = 0; i < 6; i++) chk(slv_mem(0, 8'h60 + i) == d[i], "transfer with external SCL stretching");
    chk_ev(I_ARB_LOST, 0, "clock stretching is not an arbitration loss:");
  endtask
endclass : i2c_clk_sync_vseq

// ---------------------------------------------------------------- VP-BUS-05: bus clear (4.4.10, 6.14, 8.7)
typedef enum { BC_NO_CTRL_EN, BC_SDA_RELEASE, BC_SDA_STUCK, BC_SCL_STUCK, BC_WHILE_BUSY } i2c_ip_busclr_e;

class i2c_busclr_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_busclr_vseq)
  function new(string name = "i2c_busclr_vseq"); super.new(name); endfunction

  rand i2c_ip_busclr_e kind;
  rand int unsigned    k_clocks = 4;   // BC_SDA_RELEASE: the device releases SDA after k clocks
  constraint c_def { k_clocks inside {[1:8]}; }

  task main();
    bit [31:0] v, c0;
    rd(R_CTRL, c0);
    c0 &= 32'h1C;
    quiet();
    case (kind)
      BC_NO_CTRL_EN: begin                                   // self-clears, nothing happens
        wr(R_CTRL, c0 | 32'h1);
        fork
          count_rises(60us);
          wr(R_CTRL, c0 | 32'h0002_0001);
        join
        chk(n_rise == 0, "BUSCLR with CTRL_EN = 0: no SCL pulse");
        chk_rd(R_CTRL, c0 | 32'h1, 32'h0002_001F, "CTRL.BUSCLR self-cleared");
        chk_ev(I_BUSCLR_DONE, 0, "no BUSCLR_DONE with CTRL_EN = 0:");
      end
      BC_SDA_RELEASE: begin                                  // 9 clocks + STOP whatever k, BUSCLR_DONE, bus free
        wr(R_CTRL, c0 | 32'h3);
        fork
          bb(BB_SDA_LOW_CLOCKS, k_clocks);
          begin
            #(5us);
            fork
              count_rises(300us);
              begin
                wr(R_CTRL, c0 | 32'h0002_0003);
                rd(R_STATUS, v);
                chk(v[7] && v[1], "STATUS.BUSCLR_ACTIVE / CTRL_ACTIVE during the bus clear");
                chk_rd(R_CTRL, 32'h0002_0000, 32'h0002_0000, "CTRL.BUSCLR reads 1 while active");
                wait_ev(I_BUSCLR_DONE, 1ms, 1, "bus clear");
              end
            join
          end
        join
        chk(n_rise == 10, $sformatf("9 SCL pulses + STOP clock (%0d rising edges, SDA released after %0d)", n_rise, k_clocks));
        chk_rd(R_CTRL, 0, 32'h0002_0000, "CTRL.BUSCLR cleared");
        chk_rd(R_BUSMON, 32'h3, 32'h3, "BUSMON: SCL_F = SDA_F = 1 after the bus clear");
      end
      BC_SDA_STUCK: begin                                    // ends anyway, SDA_F = 0 tells the software
        wr(R_CTRL, c0 | 32'h3);
        bb(BB_HOLD_SDA, 0, 0);
        #(5us);
        wr(R_CTRL, c0 | 32'h0002_0003);
        wait_ev(I_BUSCLR_DONE, 1ms, 1, "bus clear with SDA stuck");
        chk_rd(R_BUSMON, 32'h1, 32'h3, "BUSMON: SCL_F = 1, SDA_F = 0 (bus still stuck)");
        bb(BB_RELEASE);
      end
      BC_WHILE_BUSY: begin                                   // controller owns the bus: rejected, bit self-clears
        wr(R_CTRL, c0 | 32'h3);
        wait_dut_idle();
        push_cmds('{c_start(7'h50, 0), c_wr(8'h60)});
        wait_ev(I_CMD_STALL, 2ms, 1, "stall before BUSCLR");
        wr(R_CTRL, c0 | 32'h0002_0003);
        wait_cycles(4);
        chk_rd(R_CTRL, c0 | 32'h3, 32'h0002_001F, "CTRL.BUSCLR self-cleared (controller busy)");
        chk_rd(R_STATUS, 32'h0000_0006, 32'h0000_0086, "STATUS: no BUSCLR_ACTIVE, controller still active and holding");
        push_cmds('{c_wr(8'h61, 1)});
        wait_cmpl(0, "transfer completed after the rejected bus clear");
        chk_ev(I_BUSCLR_DONE, 0, "no BUSCLR_DONE for a rejected bus clear:");
      end
      BC_SCL_STUCK: begin                                    // blocked, the SCL timeout ends it
        bit [31:0] t0;
        rd(R_TIMEOUT, t0);
        wr(R_CTRL, 32'h0);
        reg_wr(ral.TIMEOUT, 32'h8000_0000 | 3000);
        wr(R_CTRL, c0 | 32'h3);
        bb(BB_HOLD_SCL, 0, 0);
        wr(R_CTRL, c0 | 32'h0002_0003);
        wait_ev(I_SCL_TOUT, 1ms, 1, "SCL timeout during bus clear");
        wait_ev(I_BUSCLR_DONE, 1ms, 1, "BUSCLR_DONE after the timeout");
        chk_rd(R_CTRL, 0, 32'h0002_0000, "CTRL.BUSCLR cleared by the timeout");
        bb(BB_RELEASE);
        wr(R_CTRL, 32'h0);
        reg_wr(ral.TIMEOUT, t0);
      end
    endcase
    w1c(EVENT_MASK);
    wr(R_CSTAT, 32'h7F);
    wait_bus_free(20us);
    loud();
    wr(R_CTRL, c0 | 32'h3);
    wait_dut_idle();
  endtask
endclass : i2c_busclr_vseq

// ---------------------------------------------------------------- VP-BUS-06: SCL low timeout (4.3.5, 4.4.9, 4.5.9)
//   expects i2c_ip_timeout_cfg_vseq to have enabled the timeout (TOUT_VAL << the hold time)
class i2c_tout_ctrl_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_tout_ctrl_vseq)
  function new(string name = "i2c_tout_ctrl_vseq"); super.new(name); endfunction

  rand int unsigned hold_rise = 12;    // SCL rise after which the device holds SCL
  rand int unsigned hold_ns = 70000;
  constraint c_def { hold_rise inside {[2:25]}; soft hold_ns == 70000; }

  task main();
    i2c_ip_bb_scl_stretch_vseq st = i2c_ip_bb_scl_stretch_vseq::type_id::create("st");
    int unsigned c0;
    if (!st.randomize() with { n_holds == 1; first_rise == local::hold_rise; hold_ns == local::hold_ns; }) `uvm_fatal("RAND", "bb")
    quiet();
    c0 = p_sequencer.side.ev_cnt[I_SCL_TOUT];
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(8'h01), c_wr(8'h02), c_wr(8'h03, 1)});
      run_seq(st);
    join
    chk(p_sequencer.side.ev_cnt[I_SCL_TOUT] == c0 + 1, "exactly one SCL_TOUT for one low interval");
    chk_cstat(7'h20, "SCL timeout");
    chk_ev(I_CMPL, 0, "no CMPL after the timeout abort:");
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed");
    chk_rd(R_STATUS, 0, 32'h2, "controller released the bus");
    w1c(EVENT_MASK);
    recover();
    loud();
  endtask
endclass : i2c_tout_ctrl_vseq

class i2c_tout_tgt_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_tout_tgt_vseq)
  function new(string name = "i2c_tout_tgt_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] q[$];
    bit [9:0] tq[$];
    bit ok;
    quiet();
    fork                                                     // target stretching forever (TTX never loaded)
      ext_read(own0, 2, q, ok);
      begin
        wait_ev(I_TX_REQ, 1ms, 1, "TX_REQ");
        wait_ev(I_SCL_TOUT, 1ms, 1, "timeout of the target stretch");
        wait_cycles(5);
        chk(side.d_tgt_hold === 1'b0, "target released SCL after the timeout");
      end
    join
    drain_trx(tq);
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_tout_tgt_vseq

class i2c_tout_off_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_tout_off_vseq)
  function new(string name = "i2c_tout_off_vseq"); super.new(name); endfunction

  task main();
    quiet();
    bb(BB_HOLD_SCL, 0, 60000);
    chk_ev(I_SCL_TOUT, 0, "TOUT_EN = 0: no SCL_TOUT:");
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    wait_dut_idle();
    loud();
  endtask
endclass : i2c_tout_off_vseq

// ---------------------------------------------------------------- VP-BUS-07: misplaced START / STOP (4.3.2, 4.4.9, 4.5.9)
class i2c_bus_err_tgt_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_bus_err_tgt_vseq)
  function new(string name = "i2c_bus_err_tgt_vseq"); super.new(name); endfunction

  rand int unsigned at_rise = 13;      // SCL rise (inside the data bytes) where the S/P glitch happens
  constraint c_def { soft at_rise == 13; at_rise inside {[11:26]}; at_rise % 9 != 0; }   // not on an ACK clock (SDA low)

  task main();
    i2c_ip_bb_glitch_vseq g = i2c_ip_bb_glitch_vseq::type_id::create("g");
    bit [9:0] tq[$];
    bit ok;
    if (!g.randomize() with { sda_line == 1; n == 1; rise_step == local::at_rise; delay_ns == 150; width_ns == 300; }) `uvm_fatal("RAND", "glitch")
    quiet();
    fork
      ext_write(own0, '{8'hFF, 8'hFF, 8'hFF}, ok);
      run_seq(g);
    join
    wait_ev(I_BUS_ERR, 1ms, 1, "misplaced START/STOP during target receive");
    wait_cycles(5);
    chk(side.d_tgt_hold === 1'b0 && side.d_tgt_active === 1'b0, "target idle after the bus error");
    drain_trx(tq);
    chk(tq.size() >= 1 && tq[0] == e_start(own0, 0), "address entry of the broken transaction");
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_bus_err_tgt_vseq

class i2c_bus_err_ctrl_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_bus_err_ctrl_vseq)
  function new(string name = "i2c_bus_err_ctrl_vseq"); super.new(name); endfunction

  rand int unsigned at_rise = 13;
  constraint c_def { soft at_rise == 13; at_rise inside {[11:26]}; at_rise % 9 != 0; }   // not on an ACK clock (SDA low)

  task main();
    i2c_ip_bb_glitch_vseq g = i2c_ip_bb_glitch_vseq::type_id::create("g");
    bit [31:0] v;
    if (!g.randomize() with { sda_line == 1; n == 1; rise_step == local::at_rise; delay_ns == 150; width_ns == 300; }) `uvm_fatal("RAND", "glitch")
    quiet();
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(8'hFF), c_wr(8'hFF), c_wr(8'hFF, 1)});
      run_seq(g);
    join
    wait_ev(I_BUS_ERR, 1ms, 1, "misplaced START/STOP during controller transfer");
    #(10us);
    rd(R_CSTAT, v);
    chk(v[CS_BUS_ERR], $sformatf("CSTAT.BUS_ERR (CSTAT = 0x%02h)", v));
    wr(R_CSTAT, 32'h7F);
    chk_ev(I_CMPL, 0, "no CMPL after the bus error abort:");
    chk_rd(R_STATUS, 0, 32'h2, "controller released the bus");
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed");
    w1c(EVENT_MASK);
    recover();
    loud();
  endtask
endclass : i2c_bus_err_ctrl_vseq

// ---------------------------------------------------------------- VP-BUS-08: synchronizer + spike filter (4.2)
class i2c_filter_spk_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_filter_spk_vseq)
  function new(string name = "i2c_filter_spk_vseq"); super.new(name); endfunction

  rand int unsigned spk = 5;           // SPKLEN_FS
  bit               en = 1;            // the filter also runs with EN = 0 (4.10)
  constraint c_def { soft spk == 5; spk inside {[2:20]}; }

  realtime poll_max;                   // longest interval between two BUSMON samples

  // pulse a line low for w ns on an idle bus; returns whether BUSMON.SCL_F / SDA_F followed and the delay
  // (time of the first read that returned 0, polling BUSMON back to back)
  task pulse(bit sda_line, real w, output bit seen, output realtime dly);
    realtime t0, tp;
    bit stop;
    seen = 0; dly = 0; stop = 0;
    fork
      begin
        t0 = $realtime;
        if (sda_line) bb(BB_GLITCH_SDA, 0, w); else bb(BB_GLITCH_SCL, 0, w);
        #(500ns);
        stop = 1;
      end
      begin
        bit [31:0] v;
        #(1ps);
        tp = $realtime;
        while (!stop) begin
          rd(R_BUSMON, v);
          if ($realtime - tp > poll_max) poll_max = $realtime - tp;
          tp = $realtime;
          if (!seen && v[sda_line ? 1 : 0] === 1'b0) begin seen = 1; dly = $realtime - t0; end
        end
      end
    join
  endtask

  task main();
    bit seen;
    realtime d;
    bit [31:0] f0;
    string what = $sformatf("EN = %0d, SPKLEN_FS = %0d:", en, spk);
    rd(R_FILTER, f0);
    quiet();
    wr(R_CTRL, 32'h0);
    reg_wr(ral.FILTER, {f0[31:8], 8'(spk)});
    if (en) wr(R_CTRL, 32'h1);                               // bus monitor only
    for (int l = 0; l < 2; l++) begin
      pulse(l, (spk - 1) * T + 5, seen, d);
      chk(!seen, $sformatf("%s %s spike of %0d cycles filtered", what, l ? "SDA" : "SCL", spk - 1));
      pulse(l, (spk + 2) * T + 5, seen, d);
      chk(seen, $sformatf("%s %s pulse of %0d cycles passes", what, l ? "SDA" : "SCL", spk + 2));
      // pad -> x_f: LAT = SPKLEN + 3 cycles (4.2), seen by the first BUSMON read after it: + up to one poll interval
      chk(d >= (spk + 3) * T * 1ns && d <= (spk + 4) * T * 1ns + poll_max,
          $sformatf("%s %s latency %0.0f ns within SPKLEN+3 T .. SPKLEN+4 T + poll interval %0.0f ns",
                    what, l ? "SDA" : "SCL", d / 1ns, poll_max / 1ns));
    end
    wr(R_CTRL, 32'h0);
    reg_wr(ral.FILTER, f0);
    wait_bus_free(5us);
    loud();
  endtask
endclass : i2c_filter_spk_vseq

// functional: SCL spikes (< SPKLEN) in the high phases of a write to the DUT target are ignored
class i2c_filter_traffic_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_filter_traffic_vseq)
  function new(string name = "i2c_filter_traffic_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_bb_glitch_vseq   g   = i2c_ip_bb_glitch_vseq::type_id::create("g");
    i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
    bit [7:0] d[$];
    bit [9:0] tq[$];
    bit ok;
    int unsigned maxw = (tm.spklen - 1) * T - 5;              // shorter than SPKLEN cycles
    if (!g.randomize() with { sda_line == 0; n == 12; rise_step == 3; delay_ns inside {[100:300]}; width_ns inside {[10:local::maxw]}; })
      `uvm_fatal("RAND", "glitch")
    rnd_bytes(4, d);
    quiet();
    fork
      begin ext_write(own0, d, ok); col.stop_req = 1; end
      run_seq(col);
      run_seq(g);
    join
    tq = col.q;
    chk(tq.size() == 6 && tq[0] == e_start(own0, 0) && tq[1][7:0] == d[0] && tq[4][7:0] == d[3] && tq[5] == e_stop(),
        "write received correctly in spite of the SCL spikes");
    chk_ev(I_BUS_ERR, 0, "no bus error from filtered spikes:");
    wait_bus_free(10us);
    loud();
  endtask
endclass : i2c_filter_traffic_vseq

// ---------------------------------------------------------------- VP-BUS-09: BUSMON / override (4.10, 6.14 fallback)
class i2c_busmon_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_busmon_vseq)
  function new(string name = "i2c_busmon_vseq"); super.new(name); endfunction

  task ovrd(bit [2:0] v, bit [1:0] exp_lines, string what);
    wr(R_BUSMON, {21'h0, v, 8'h0});
    wait_cycles(20);
    chk({bus.sda, bus.scl} === exp_lines, $sformatf("%s: bus SDA/SCL = %b%b", what, bus.sda, bus.scl));
    chk_rd(R_BUSMON, {21'h0, v, 4'h0, ~exp_lines[1] & v[0], ~exp_lines[0] & v[0], exp_lines}, 32'h7FF, {what, " BUSMON"});
  endtask

  task main();
    quiet();
    wr(R_CTRL, 32'h0);
    chk_rd(R_BUSMON, 32'h603, 32'hFFFF_FFFF, "BUSMON idle");
    // bit-bang through the override (EN = 0): SCL low, SDA low, SCL high, SDA high (STOP)
    ovrd(3'b101, 2'b10, "override SCL low");
    ovrd(3'b001, 2'b00, "override SCL + SDA low");
    ovrd(3'b011, 2'b01, "override SDA low");
    ovrd(3'b111, 2'b11, "override released (STOP)");
    ovrd(3'b000, 2'b11, "OVRD_EN = 0: override values ignored");
    // SCL_F / SDA_F follow the bus (external device) with EN = 0
    bb(BB_HOLD_SDA, 0, 0);
    wait_cycles(20);
    chk_rd(R_BUSMON, 32'h1, 32'hF, "BUSMON.SDA_F = 0 when a device pulls SDA (EN = 0)");
    bb(BB_RELEASE);
    bb(BB_HOLD_SCL, 0, 0);
    wait_cycles(20);
    chk_rd(R_BUSMON, 32'h2, 32'hF, "BUSMON.SCL_F = 0 when a device pulls SCL (EN = 0)");
    bb(BB_RELEASE);
    // EN = 1: the override is ignored
    wr(R_BUSMON, 32'h0000_0100);                              // OVRD_EN, SCL_OVRD = 0, SDA_OVRD = 0
    wr(R_CTRL, 32'h1);
    wait_cycles(20);
    chk(bus.scl === 1'b1 && bus.sda === 1'b1 && side.scl_oe === 1'b0, "override ignored while EN = 1");
    wr(R_CTRL, 32'h0);
    wait_cycles(20);
    chk(bus.scl === 1'b0 && bus.sda === 1'b0, "override active again with EN = 0");
    wr(R_BUSMON, 32'h0000_0600);
    wait_bus_free(10us);
    loud();
  endtask
endclass : i2c_busmon_vseq

// ---------------------------------------------------------------- VP-BUS-10: bus busy / free, tBUF (4.3.3, R7)
//   expects EN = 0 at the start and BUSIDLE.IDLE_VAL = 1000
class i2c_bus_busy_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_bus_busy_vseq)
  function new(string name = "i2c_bus_busy_vseq"); super.new(name); endfunction

  task main();
    bit ok;
    realtime t_stop, t_free, t_start;
    wr(R_CTRL, 32'h0);
    reg_wr(ral.BUSIDLE, 1000);
    // EN 0 -> 1: BUS_BUSY = 1 until IDLE_VAL cycles of idle bus (R7)
    wr(R_CTRL, 32'h1);
    chk_rd(R_STATUS, 1, 1, "BUS_BUSY right after enable");
    #(5us);
    chk_rd(R_STATUS, 1, 1, "BUS_BUSY 5 us after enable (IDLE_VAL = 10 us)");
    #(8us);
    chk_rd(R_STATUS, 0, 1, "bus free after the idle detection");
    // external transaction: busy between S and P (bus free only after TBUF: DUT START check below)
    fork
      ext_write(7'h50, '{8'h00, 8'h01}, ok);
      begin
        wait (side.d_bus_busy === 1'b1);
        chk_rd(R_STATUS, 1, 1, "BUS_BUSY during an external transaction");
        @(posedge bus.sda iff bus.scl === 1'b1);
        t_stop = $realtime;
        wait (side.d_bus_busy === 1'b0);
        t_free = $realtime;
      end
    join
    chk(t_free - t_stop < 20 * T * 1ns, $sformatf("debug_bus_busy_o falls %0.0f ns after STOP", (t_free - t_stop) / 1ns));
    chk_rd(R_STATUS, 0, 1, "BUS_BUSY = 0 after STOP");
    // DUT controller START waits TBUF after an external STOP
    wr(R_CTRL, 32'h3);
    fork
      ext_write(7'h51, '{8'h00, 8'h02}, ok);
      begin
        wait (side.d_bus_busy === 1'b1);
        push_cmds('{c_start(7'h50, 0), c_wr(8'h00, 1)});
        @(posedge bus.sda iff bus.scl === 1'b1);
        t_stop = $realtime;
        @(negedge bus.sda iff bus.scl === 1'b1);
        t_start = $realtime;
      end
    join
    wait_cmpl(0, "controller after an external transaction");
    chk(t_start - t_stop >= tm.tbuf * T * 1ns, $sformatf("DUT START %0.0f ns after the external STOP >= TBUF", (t_start - t_stop) / 1ns));
    // enable while SDA is held low: no idle detection; smaller IDLE_VAL after release
    wr(R_CTRL, 32'h0);
    reg_wr(ral.BUSIDLE, 300);
    wait_bus_free(5us);
    quiet();                                                // SDA pulled with SCL high looks like a START
    bb(BB_HOLD_SDA, 0, 0);
    wr(R_CTRL, 32'h1);
    #(20us);
    chk_rd(R_STATUS, 1, 1, "BUS_BUSY stays while SDA is low");
    bb(BB_RELEASE);
    #(4us);
    chk_rd(R_STATUS, 0, 1, "bus free 3 us (IDLE_VAL = 300) after the release");
    wr(R_CTRL, 32'h0);
    reg_wr(ral.BUSIDLE, 1000);
    loud();
  endtask
endclass : i2c_bus_busy_vseq

// ---------------------------------------------------------------- VP-BUS-11: software reset (5.2.1, 6.1.2)
class i2c_swrst_regs_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_swrst_regs_vseq)
  function new(string name = "i2c_swrst_regs_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] kept_v[$];
    bit [11:0] kept_a[$];
    bit ok;
    // registers SWRST must keep
    wr(R_CTRL, 32'h0);
    set_timing(SPD_FMP);
    reg_wr(ral.HS_TIM_SCL, 32'h0006_0010);
    reg_wr(ral.BUSIDLE, 1200);
    reg_wr(ral.TCFG, 32'h9);                                  // GC_EN | DEVID_EN
    wr(R_CTRL, 32'h0000_001D);                               // EN, TGT_EN, HS_EN, STARTBYTE_EN (controller off)
    wait_dut_idle();
    wr(R_INTR_EN, 32'h0000_003F);
    wr(R_FIFO_CTRL, 32'h0102_0304 & {4{8'(depth - 1)}});
    wr(R_HSCFG, 5);
    wr(R_TIMEOUT, 32'h8012_3456);
    wr(R_DEVID, 32'h0065_4321);
    wr(R_BUSMON, 32'h0000_0200);
    wr(R_DMA_BLK, 3);
    kept_a = '{R_INTR_EN, R_FIFO_CTRL, R_TADDR, R_TCFG, R_HSCFG, R_TIM_SCL, R_TIM_STA, R_TIM_STO, R_TIM_DAT,
               R_HS_TIM_SCL, R_FILTER, R_TIMEOUT, R_BUSIDLE, R_DEVID, R_DMA_BLK};
    foreach (kept_a[i]) begin bit [31:0] v; rd(kept_a[i], v); kept_v.push_back(v); end
    // state SWRST must clear
    ext_write(own0, '{8'h01, 8'h02}, ok);                    // TRX entries, TSTAT
    wr(R_CTRL, 32'h0001_001D);                               // idle ABORT -> CSTAT.USER_ABORT
    for (int i = 0; i < 3; i++) wr(R_CMD, c_wr(i));          // CMD (controller disabled)
    load_ttx('{8'h11, 8'h22});
    wr(R_INTR_TEST, EVENT_MASK);
    wr(R_DMA_CTRL, 32'h0000_0027);                           // SEL = TTX, PFC, END_EN, EN: request -> DMAC DENY
    #(2us);
    chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0000, "DMA_STAT.DENY before SWRST");
    p_sequencer.dmac.send_end(1);                            // DMAC end pulse: END, and DONE (END_EN = 1, EN = 1)
    chk_rd(R_DMA_STAT, 32'h0101_0100, 32'h0101_0100, "DMA_STAT.DONE/DENY/END before SWRST");
    chk_rd(R_CSTAT, 32'h08, '1, "CSTAT before SWRST");
    // SWRST through byte lane 3 only (CTRL[4:0] not written by this access)
    wr(R_CTRL, 32'h8000_0000, 4'b1000);
    chk_rd(R_CTRL, 32'h0000_0018, '1, "CTRL after SWRST: EN/CTRL_EN/TGT_EN cleared, HS_EN/STARTBYTE_EN kept");
    chk_rd(R_FIFO_STAT, 0, '1, "all FIFOs flushed");
    chk_rd(R_INTR_STAT, 32'h0000_0201, '1, "event bits cleared (level bits CMD_THR/TTX_THR remain)");
    chk_rd(R_CSTAT, 0, '1, "CSTAT cleared");
    chk_rd(R_TSTAT, 0, '1, "TSTAT cleared");
    chk_rd(R_DMA_CTRL, 32'h0000_0026, '1, "DMA_CTRL.EN cleared, SEL/PFC/END_EN kept");
    chk_rd(R_DMA_STAT, 0, 32'h0101_0100, "DMA_STAT.DONE/DENY/END cleared");
    chk_rd(R_BUSMON, 32'h0000_0200, 32'h0000_0700, "BUSMON[10:8] kept");
    foreach (kept_a[i]) chk_rd(kept_a[i], kept_v[i], '1, $sformatf("register 0x%03h kept by SWRST", kept_a[i]));
    chk(side.scl_oe === 1'b0 && side.sda_oe === 1'b0, "SCL/SDA released");
    p_sequencer.sb.resync();
    wr(R_BUSMON, 32'h0000_0600);
    wr(R_DMA_CTRL, 32'h0);
    wr(R_DMA_BLK, 0);
    wr(R_FIFO_CTRL, 0);
    wr(R_INTR_EN, 0);
    reg_wr(ral.TCFG, 0);
    wr(R_CTRL, 32'h0);
  endtask
endclass : i2c_swrst_regs_vseq

// SWRST in the middle of a controller transfer: bus released at once, no CMPL, recovery by bus clear
class i2c_swrst_active_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_swrst_active_vseq)
  function new(string name = "i2c_swrst_active_vseq"); super.new(name); endfunction

  rand int unsigned at_bit = 4;
  constraint c_def { soft at_bit == 4; at_bit inside {[1:8]}; }

  task main();
    quiet();
    push_cmds('{c_start(7'h50, 0), c_wr(8'h01), c_wr(8'h02), c_wr(8'h03), c_wr(8'h04, 1)});
    wait (side.b_bit_cnt == at_bit && side.d_ctrl_active === 1'b1);
    wr(R_CTRL, 32'h8000_0000);
    wait_cycles(3);
    chk(side.scl_oe === 1'b0 && side.sda_oe === 1'b0, "SWRST releases SCL/SDA immediately");
    chk_rd(R_CTRL, 0, 32'h7, "CTRL[2:0] cleared");
    chk_ev(I_CMPL, 0, "no CMPL after SWRST:");
    wr(R_CTRL, 32'h3);
    recover();
    loud();
  endtask
endclass : i2c_swrst_active_vseq

// ---------------------------------------------------------------- VP-BUS-12: EN / TGT_EN while active (4.1, 4.5.9, 6.16)
class i2c_en_ctrl_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_en_ctrl_vseq)
  function new(string name = "i2c_en_ctrl_vseq"); super.new(name); endfunction

  task main();
    int unsigned lvl;
    quiet();
    push_cmds('{c_start(7'h50, 0), c_wr(8'h01), c_wr(8'h02), c_wr(8'h03), c_wr(8'h04), c_wr(8'h05), c_wr(8'h06, 1)});
    do begin
      wait (side.d_ctrl_active === 1'b1);
      rd_field(R_FIFO_STAT, 0, 6, lvl);
    end while (lvl > 4);
    wr(R_CTRL, 32'h0);
    wait_cycles(3);
    chk(side.scl_oe === 1'b0 && side.sda_oe === 1'b0, "EN = 0 releases SCL/SDA");
    rd_field(R_FIFO_STAT, 0, 6, lvl);
    chk(lvl > 0, $sformatf("CMD FIFO content kept with EN = 0 (%0d entries)", lvl));
    wr(R_CTRL, 32'h0000_0100);
    wr(R_CTRL, 32'h3);
    recover();
    loud();
  endtask
endclass : i2c_en_ctrl_vseq

class i2c_tgt_en_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_tgt_en_vseq)
  function new(string name = "i2c_tgt_en_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
    bit [7:0] d[$];
    bit [9:0] tq[$];
    bit ok;
    rnd_bytes(6, d);
    // TGT_EN -> 0 inside a transaction: the transaction completes, later ones are NACKed
    fork
      begin ext_write(own0, d, ok); col.stop_req = 1; end
      begin
        wait (side.d_tgt_active === 1'b1);
        wr(R_CTRL, 32'h1);
        run_seq(col);
      end
    join
    chk(ok, "transaction in progress completes after TGT_EN = 0");
    chk(col.q.size() == 8, $sformatf("all entries of the completed transaction (%0d)", col.q.size()));
    ext_write(own0, '{8'h01}, ok);
    chk(!ok, "no answer after TGT_EN = 0");
    // TGT_EN 0 -> 1 is effective at the next START
    wr(R_CTRL, 32'h5);
    ext_write(own0, '{8'h02}, ok);
    chk(ok, "TGT_EN = 1 again");
    drain_trx(tq);
  endtask
endclass : i2c_tgt_en_vseq

// ---------------------------------------------------------------- VP-BUS-13: arbitration lost while sending a repeated START
//   both controllers write the same address and pointer; the DUT then sends Sr, the other one a data bit 0:
//   SDA is low when SCL goes high during the Sr setup (4.4.6)
class i2c_arb_lost_sr_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_sr_vseq)
  function new(string name = "i2c_arb_lost_sr_vseq"); super.new(name); endfunction

  rand bit [7:0] ptr, wd;
  constraint c_def { wd[7] == 0; }                         // first bit 0 collides with the Sr setup

  task main();
    i2c_ip_bb_arb_vseq bbv = i2c_ip_bb_arb_vseq::type_id::create("bbv");
    bbv.bytes = '{{7'h50, 1'b0}, ptr, wd};
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(ptr), c_start(7'h50, 1), c_rd(0, 1)});
      run_seq(bbv);
    join
    chk(!bbv.lost && bbv.acks[2], "the other controller wins and writes its data byte");
    wait_ev(I_ARB_LOST, 1ms, 1, "arbitration lost in the Sr");
    #(5us);
    chk_ev(I_CMPL, 0, "no CMPL after an arbitration loss in the Sr:");
    chk_cstat(7'h04, "arbitration lost in the Sr");
    chk_rd(R_FIFO_STAT, 0, 32'h3F3F, "CMD flushed, nothing read");
    chk(slv_mem(0, ptr) == wd, "the external target got the byte of the winner");
  endtask
endclass : i2c_arb_lost_sr_vseq

// ---------------------------------------------------------------- VP-BUS-14: arbitration lost on the NACK of a read
//   both controllers read the same target; the DUT NACKs its only byte, the other one ACKs it (4.4.6, 2nd rule)
class i2c_arb_lost_nack_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_nack_vseq)
  function new(string name = "i2c_arb_lost_nack_vseq"); super.new(name); endfunction

  // spec 4.4.6 (5) / 6.13: the byte received before a loss at the NACK slot stays in CRX with LAST = 1 and the
  // software drains it. The reference model does not predict this byte: the data path is checked here
  // (scoreboard off around the abuse, re-synchronised after)
  task main();
    bit [7:0] b0, b1;
    bit ack, lost;
    bit [31:0] v;
    quiet();
    fork
      push_cmds('{c_start(7'h50, 1), c_rd(0, 1)});
      begin
        bb(BB_JOIN_START);
        p_sequencer.bbs.tx({7'h50, 1'b1}, ack, lost);
        chk(!lost && ack, "same read address: no loss, target ACK");
        p_sequencer.bbs.rx(b0, 1);                           // ACK: the DUT sending NACK loses here
        p_sequencer.bbs.rx(b1, 0);
        bb(BB_STOP);
      end
    join
    wait_ev(I_ARB_LOST, 1ms, 1, "arbitration lost on the NACK bit");
    #(5us);
    chk_ev(I_CMPL, 0, "no CMPL after an arbitration loss on the NACK:");
    chk_cstat(7'h04, "arbitration lost on the NACK");
    // the byte was pushed at the SCL rise of bit 8, before the NACK slot (4.4.4)
    chk_rd(R_FIFO_STAT, 1 << 8, 32'h3F3F, "CRX holds the byte read before the loss, CMD flushed");
    rd(R_CRXDATA, v);
    chk(v[7:0] == b0 && v[8], $sformatf("CRX byte 0x%03h = first byte 0x%02h with LAST", v[8:0], b0));
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_arb_lost_nack_vseq

// ---------------------------------------------------------------- VP-BUS-15: SDA held low after the STOP release
//   SDA does not rise within SCL_LOW + SCL_HIGH + TSU_STO cycles: arbitration lost, no hang, no CMPL (4.4.3, 4.4.6)
class i2c_arb_lost_stop_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_stop_vseq)
  function new(string name = "i2c_arb_lost_stop_vseq"); super.new(name); endfunction

  rand int unsigned hold_ns;
  constraint c_def { hold_ns inside {[10000:40000]}; }

  task main();
    bit [7:0] d = rnd8();
    quiet();
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(8'h70), c_wr(d, 1)});
      begin
        bb(BB_WAIT_START);
        bb(BB_WAIT_SCL_RISE, 28);                            // STOP clock: 3 bytes x 9 clocks + 1
        bb(BB_HOLD_SDA, 0, hold_ns);                         // SDA kept low while SCL is high
      end
    join
    chk(p_sequencer.side.ev_cnt[I_ARB_LOST] > 0, "ARB_LOST when SDA does not rise for the STOP");
    wait_ev(I_ARB_LOST, 1ms, 1, "STOP watchdog");
    chk_ev(I_CMPL, 0, "no CMPL after the STOP watchdog:");
    chk_cstat(7'h04, "STOP watchdog");
    chk(slv_mem(0, 8'h70) == d, "the data byte was delivered before the STOP");
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_arb_lost_stop_vseq

// ---------------------------------------------------------------- VP-BUS-16: SCL pulled low while the DUT sends Sr / STOP
//   another device pulls SCL low before the Sr setup / STOP setup time has elapsed: arbitration lost (4.4.6, 4th rule),
//   no CMPL; the bus is recovered with a bus clear (6.14)
typedef enum { SP_RSTART, SP_STOP } i2c_ip_sp_e;

class i2c_arb_lost_sp_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_lost_sp_vseq)
  function new(string name = "i2c_arb_lost_sp_vseq"); super.new(name); endfunction

  rand i2c_ip_sp_e  kind;
  rand int unsigned delay_ns;          // after the SCL rise of the Sr / STOP clock (< TSU_STA / TSU_STO)
  constraint c_def { delay_ns inside {[60:250]}; }

  task main();
    i2c_ip_bb_glitch_vseq g = i2c_ip_bb_glitch_vseq::type_id::create("g");
    bit [12:0] c[$];
    if (!g.randomize() with { sda_line == 0; n == 1; rise_step == 19; delay_ns == local::delay_ns; width_ns == 300; })
      `uvm_fatal("RAND", "glitch")
    if (kind == SP_RSTART) c = '{c_start(7'h50, 0), c_wr(8'h30), c_start(7'h50, 1), c_rd(0, 1)};   // rise 19 = Sr clock
    else                   c = '{c_start(7'h50, 0), c_wr(8'h30, 1)};                             // rise 19 = STOP clock
    quiet();
    fork
      push_cmds(c);
      run_seq(g);
    join
    wait_ev(I_ARB_LOST, 1ms, 1, $sformatf("SCL low during the %s", kind.name()));
    #(5us);
    chk_ev(I_CMPL, 0, $sformatf("no CMPL after the loss in the %s:", kind.name()));
    chk_cstat(7'h04, kind.name());
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed");
    chk_rd(R_STATUS, 0, 32'h2, "controller released the bus");
    w1c(EVENT_MASK);
    recover();
    loud();
  endtask
endclass : i2c_arb_lost_sp_vseq

// ---------------------------------------------------------------- VP-BUS-17: SCL pulled low around START / Sr / STOP
//   spec 4.4.3 table, 4.4.6:
//   AH_START_SYNC / AH_RSTART_SYNC : SCL pulled low inside the START / Sr hold while SDA is low = clock
//                                    synchronization: the transfer goes on, no ARB_LOST
//   AH_START_SCL_LOW               : SCL already low when the DUT starts (START not visible): ARB_LOST in C_STA
//   AH_STOP_WAIT                   : SDA held low by another device across the STOP, then SCL low: ARB_LOST
typedef enum { AH_START_SYNC, AH_RSTART_SYNC, AH_START_SCL_LOW, AH_STOP_WAIT } i2c_ip_hold_e;

class i2c_arb_hold_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_arb_hold_vseq)
  function new(string name = "i2c_arb_hold_vseq"); super.new(name); endfunction

  rand i2c_ip_hold_e kind;
  rand int unsigned  pos;              // SCL pulse position inside the START / Sr hold, % of THD_STA
  constraint c_def { pos inside {[5:85]}; }

  real lat_ns;                         // input latency of the DUT (spec 4.2)

  task main();
    lat_ns = (tm.spklen + 3) * T;
    case (kind)
      AH_START_SYNC, AH_RSTART_SYNC: hold_sync();
      AH_START_SCL_LOW:              start_scl_low();
      default:                       stop_wait();
    endcase
  endtask

  // the pulse must outlast the reaction of the DUT (LAT + 2 cycles), else SCL would bounce (extra clock)
  function int unsigned pulse_ns();
    return int'(lat_ns + 6 * T + 100);
  endfunction

  task hold_sync();
    i2c_ip_bb_start_pulse_vseq p = i2c_ip_bb_start_pulse_vseq::type_id::create("p");
    bit [7:0] d[$], q[$];
    bit [7:0] ptr = 8'h20 + 8'($urandom_range(0, 15));
    int unsigned dly = int'(2 * T + tm.thd_sta * T * pos / 100.0);
    int unsigned wid = pulse_ns();
    rnd_bytes(3, d);
    if (!p.randomize() with { nth == ((local::kind == AH_START_SYNC) ? 1 : 2);
                              delay_ns == local::dly; width_ns == local::wid; })
      `uvm_fatal("RAND", "start pulse")
    if (kind == AH_RSTART_SYNC) ctrl_write(7'h50, '{ptr, d[0], d[1], d[2]});      // data to read back
    w1c(EVENT_MASK);
    quiet();
    if (kind == AH_START_SYNC) begin
      fork
        ctrl_write(7'h50, '{ptr, d[0], d[1], d[2]});
        run_seq(p);
      join
      chk(slv_mem(0, ptr) == d[0] && slv_mem(0, ptr + 1) == d[1] && slv_mem(0, ptr + 2) == d[2],
          $sformatf("write with SCL pulled low %0d ns into the START hold reached the slave", dly));
    end else begin
      fork
        ctrl_write_read(7'h50, '{ptr}, 3, q);
        run_seq(p);
      join
      chk(q == d, $sformatf("register read with SCL pulled low %0d ns into the Sr hold", dly));
    end
    chk_ev(I_ARB_LOST, 0, $sformatf("%s: clock synchronization, not an arbitration loss:", kind.name()));
    loud();
  endtask

  task start_scl_low();
    wait_bus_free(20us);
    w1c(EVENT_MASK);
    quiet();
    bb(BB_HOLD_SCL);                                         // no START: the DUT still sees a free bus (4.3.3)
    push_cmds('{c_start(7'h50, 0), c_wr(8'h11, 1)});
    wait_ev(I_ARB_LOST, 200us, 1, "START of the DUT with SCL already low (SDA not yet low on the bus)");
    #(2us);
    chk_ev(I_CMPL, 0, "no CMPL after the loss in C_STA:");
    chk_cstat(7'h04, "START with SCL held low");
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed");
    chk_rd(R_STATUS, 0, 32'h2, "controller released the bus");
    chk(bus.sda === 1'b1, "SDA released after the loss");
    bb(BB_RELEASE);
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    loud();
  endtask

  task stop_wait();
    i2c_ip_bb_stop_block_vseq b = i2c_ip_bb_stop_block_vseq::type_id::create("b");
    // the DUT releases SDA TSU_STO cycles after it observes the SCL rise of the STOP clock (+LAT, +1 output flop)
    int unsigned aft = int'((tm.tsu_sto + 4) * T + 50);
    int unsigned wid = pulse_ns();
    if (!b.randomize() with { rise == 19;                    // S + address + ACK (9) + data + ACK (9): rise 19
                              pulse_after_ns == local::aft; width_ns == local::wid; })
      `uvm_fatal("RAND", "stop block")
    w1c(EVENT_MASK);
    quiet();
    fork
      push_cmds('{c_start(7'h50, 0), c_wr(8'h33, 1)});
      run_seq(b);
    join
    wait_ev(I_ARB_LOST, 1ms, 1, "SCL low after the DUT released SDA for its STOP (SDA held by another device)");
    #(5us);
    chk_ev(I_CMPL, 0, "no CMPL after the loss in C_P_W:");
    chk_cstat(7'h04, "STOP blocked by a data bit 0");
    chk_rd(R_STATUS, 0, 32'h2, "controller released the bus");
    w1c(EVENT_MASK);
    wait_bus_free(20us);                                      // the other device ended with a STOP
    loud();
  endtask
endclass : i2c_arb_hold_vseq

// ---------------------------------------------------------------- VP-BUS-18: misplaced START / STOP, target transmitter
//   spec 4.5.10: S / P while the target sends a 1 bit (clock bit_k of the data byte) or in the ACK clock of the
//   controller (after NACK: S, after ACK: P). The external controller is the bit-bang agent.
typedef enum { MS_TX_START, MS_TX_STOP, MS_ACK_START, MS_ACK_STOP } i2c_ip_mis_e;

class i2c_bus_err_ttx_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_bus_err_ttx_vseq)
  function new(string name = "i2c_bus_err_ttx_vseq"); super.new(name); endfunction

  rand i2c_ip_mis_e kind;
  rand int unsigned bit_k;             // clock of the data byte with the S / P (1..8, 9 = ACK clock)
  constraint c_def { (kind inside {MS_ACK_START, MS_ACK_STOP})  -> bit_k == 9;
                     !(kind inside {MS_ACK_START, MS_ACK_STOP}) -> bit_k inside {[1:8]}; }

  task main();
    bit [7:0]    d[$], v, w;
    bit [9:0]    tq[$], exp[$];
    bit          ack, lost, is_start;
    bit [31:0]   c;
    int unsigned nb = (bit_k <= 8) ? bit_k - 1 : 8;         // data bits clocked before the S / P
    int unsigned m  = (depth < 4) ? depth - 1 : 3;          // TTX bytes behind the interrupted one
    is_start = (kind inside {MS_TX_START, MS_ACK_START});
    w = rnd8();
    rnd_bytes(1 + m, d);
    if (bit_k <= 8) d[0][8 - bit_k] = 1'b1;                 // the target releases SDA at the S / P clock
    reg_wr(ral.TCFG, 0);
    load_ttx(d);
    w1c(EVENT_MASK);
    quiet();
    p_sequencer.bbs.start_c();
    p_sequencer.bbs.tx({own0, 1'b1}, ack, lost);
    chk(ack, "DUT target ACKs its read address");
    p_sequencer.bbs.rx_bits(nb, v);
    chk(v == 8'(d[0] >> (8 - nb)), $sformatf("%0d bit(s) of 0x%02h sent before the S / P (got 0x%02h)", nb, d[0], v));
    if (is_start) begin
      p_sequencer.bbs.rstart_c();                            // START while the target releases SDA
      p_sequencer.bbs.tx({own0, 1'b0}, ack, lost);
      chk(ack, "address after the misplaced START is ACKed (new address reception)");
      p_sequencer.bbs.tx(w, ack, lost);
      chk(ack, "data after the misplaced START");
      p_sequencer.bbs.stop_c();
      exp = '{e_start(own0, 1), e_rstart(own0, 0), e_data(w), e_stop()};
    end else begin
      p_sequencer.bbs.stop_c();                              // STOP: SDA rises while the target releases it
      exp = '{e_start(own0, 1), e_stop()};
    end
    ext_settle();
    if (bit_k >= 2) wait_ev(I_BUS_ERR, 100us, 0, $sformatf("%s at clock %0d: BUS_ERR", kind.name(), bit_k));
    else            chk_ev(I_BUS_ERR, 0, "S / P at the first clock of a byte is not a bus error:");
    chk_ev(I_TTX_DONE, 0, "no TTX_DONE after a misplaced S / P:");
    chk_ev(I_TSTOP, 1, "TSTOP at the STOP of the transaction:");
    chk(side.d_tgt_active === 1'b0 && side.d_tgt_hold === 1'b0, "target idle after the transaction");
    drain_trx(tq);
    chk_trx(tq, exp, $sformatf("%s at clock %0d", kind.name(), bit_k));
    chk_rd(R_FIFO_STAT, m << 16, 32'h3F << 16, "TTX: the interrupted byte is not re-sent, the later bytes are kept");
    rd(R_CTRL, c);
    wr(R_CTRL, (c & 32'h1F) | (1 << 10));                   // recovery of 4.5.10: TTX_FLUSH
    chk_rd(R_FIFO_STAT, 0, 32'h3F << 16, "TTX flushed");
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_bus_err_ttx_vseq
