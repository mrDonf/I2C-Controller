// -----------------------------------------------------------------------------
// Layer 2 - target feature sequences (spec 4.5, 4.6, 6.10, 6.11)
//   The VIP master is the external controller. TRX content / TTX bytes / Device ID bytes / address ACKs are checked by
//   the scoreboard on every message; these sequences additionally compare explicit TRX entry lists, events, TSTAT and
//   STATUS against the spec. They expect the target to be enabled (CTRL.TGT_EN) by the test.
// -----------------------------------------------------------------------------

class i2c_ip_tgt_base_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_tgt_base_vseq)
  function new(string name = "i2c_ip_tgt_base_vseq"); super.new(name); endfunction

  // external write with a concurrent TRX collector; returns the entries
  task ext_write_col(bit [6:0] a, bit [7:0] d[$], output bit ok, output bit [9:0] q[$], input bit stop = 1);
    i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
    fork
      begin ext_write(a, d, ok, stop); col.stop_req = 1; end
      run_seq(col);
    join
    q = col.q;
  endtask

  // external read served by the TX_REQ routine (src loaded on demand); returns data and the TRX entries
  task ext_read_srv(bit [6:0] a, bit [7:0] src[$], int unsigned n, output bit [7:0] q[$], output bit ok,
                    output bit [9:0] rx[$], input int ptr = -1);
    i2c_ip_ttx_service_vseq srv = i2c_ip_ttx_service_vseq::type_id::create("srv");
    bit [9:0] tq[$];
    srv.src = src;
    fork
      run_seq(srv);
      begin
        if (ptr < 0) ext_read(a, n, q, ok); else ext_write_read(a, ptr, n, q, ok);
        srv.stop_req = 1;
      end
    join
    drain_trx(tq);
    rx = srv.rx_log;
    foreach (tq[i]) rx.push_back(tq[i]);
  endtask
endclass : i2c_ip_tgt_base_vseq

// ---------------------------------------------------------------- VP-TGT-01: target receiver (4.5.1 - 4.5.3, 6.10)
class i2c_tgt_write_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_write_vseq)
  function new(string name = "i2c_tgt_write_vseq"); super.new(name); endfunction

  rand bit          use_addr1;
  rand int unsigned n = 3;
  constraint c_def { n inside {[1:16]}; soft n inside {[1:4]}; }

  task main();
    i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
    bit [7:0] d[$];
    bit [9:0] exp[$];
    bit [6:0] a = use_addr1 ? own1 : own0;
    bit [31:0] en0;
    bit ok;
    rnd_bytes(n, d);
    rd(R_INTR_EN, en0);
    wr(R_INTR_EN, 32'h0000_1800);                            // TADDR | TSTOP
    w1c(EVENT_MASK);
    fork
      begin ext_write(a, d, ok); col.stop_req = 1; end
      begin
        wait (side.d_tgt_active === 1'b1);
        chk_rd(R_STATUS, 32'h0000_0009, 32'h0000_0019, "STATUS: BUS_BUSY, TGT_ACTIVE, TGT_RW = 0 during the write");
        run_seq(col);
      end
    join
    chk(ok, $sformatf("write to ADDR%0d acknowledged", use_addr1));
    chk_ev(I_TADDR, 1, "TADDR:"); chk_ev(I_TSTOP, 1, "TSTOP:");
    chk(side.irq_tgt === 1'b1, "irq_tgt_o with TADDR / TSTOP enabled");
    w1c(EVENT_MASK);
    wait_cycles(3);
    chk(side.irq_tgt === 1'b0, "irq_tgt_o released after the W1C");
    chk_rd(R_TSTAT, {21'h0, 1'b0, use_addr1, 1'b0, 1'b0, a}, '1, $sformatf("TSTAT after ADDR%0d write", use_addr1));
    chk_rd(R_STATUS, 0, 32'h0000_0019, "STATUS: target idle after STOP");
    exp.push_back(e_start(a, 0));
    foreach (d[i]) exp.push_back(e_data(d[i]));
    exp.push_back(e_stop());
    chk_trx(col.q, exp, $sformatf("ADDR%0d write of %0d bytes", use_addr1, n));
    wr(R_INTR_EN, en0);
  endtask
endclass : i2c_tgt_write_vseq

// ---------------------------------------------------------------- VP-TGT-02: repeated START tags (4.5.2)
class i2c_tgt_rstart_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_rstart_vseq)
  function new(string name = "i2c_tgt_rstart_vseq"); super.new(name); endfunction

  task main();
    bit [9:0] q[$], q2[$];
    bit ok;
    // S ADDR0 W d Sr ADDR1 W d P -> START tag then Sr tag (11)
    ext_write_col(own0, '{8'h11}, ok, q, 0);
    ext_write_col(own1, '{8'h22}, ok, q2);
    foreach (q2[i]) q.push_back(q2[i]);
    chk_trx(q, '{e_start(own0, 0), e_data(8'h11), e_rstart(own1, 0), e_data(8'h22), e_stop()}, "Sr inside one transaction");
    // first address of the transaction to another device, then Sr to us: tag 11 (a real address came first)
    ext_write_col(7'h50, '{8'h00}, ok, q, 0);
    ext_write_col(own0, '{8'h33}, ok, q2);
    foreach (q2[i]) q.push_back(q2[i]);
    chk_trx(q, '{e_rstart(own0, 0), e_data(8'h33), e_stop()}, "Sr after an address of another device");
    // Sr to the same address, write then read (pointer write + read in one transaction)
    load_ttx('{8'hC1, 8'hC2});
    begin
      bit [7:0] rq[$];
      ext_write_read(own1, 8'h07, 2, rq, ok);
      chk(ok && rq[0] == 8'hC1 && rq[1] == 8'hC2, "write-read with Sr to the same address");
    end
    drain_trx(q);
    chk_trx(q, '{e_start(own1, 0), e_data(8'h07), e_rstart(own1, 1), e_stop()}, "write-read TRX entries");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_rstart_vseq

// ---------------------------------------------------------------- VP-TGT-03: address enables / foreign addresses (4.5.1)
class i2c_tgt_addr_filter_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_addr_filter_vseq)
  function new(string name = "i2c_tgt_addr_filter_vseq"); super.new(name); endfunction

  task nack_case(bit [6:0] a, bit rd_, string what);
    bit [7:0] q[$];
    bit ok;
    if (rd_) ext_read(a, 1, q, ok); else ext_write(a, '{8'h06}, ok);
    chk(!ok, $sformatf("%s (0x%02h %s) not acknowledged", what, a, rd_ ? "R" : "W"));
  endtask

  task main();
    bit [9:0] tq[$];
    bit [6:0] foreign;
    bit ok;
    w1c(EVENT_MASK);
    do foreign = 7'($urandom_range(8, 7'h77)); while (foreign inside {own0, own1, 7'h50, 7'h51});
    nack_case(foreign, $urandom_range(1), "foreign address");
    chk_ev(I_TADDR, 0, "no TADDR for a foreign address:");
    chk_rd(R_FIFO_STAT, 0, '1, "TRX empty");
    // ADDR1_EN = 0 / ADDR0_EN = 0 (configuration latched at START)
    reg_wr(ral.TADDR, {16'h0, 1'b0, own1, 1'b1, own0});
    nack_case(own1, 0, "ADDR1_EN = 0");
    ext_write(own0, '{8'h01}, ok);
    chk(ok, "ADDR0 still answered with ADDR1_EN = 0");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own0, 0), e_data(8'h01), e_stop()}, "ADDR0 with ADDR1_EN = 0");
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b0, own0});
    nack_case(own0, 1, "ADDR0_EN = 0");
    ext_write(own1, '{8'h02}, ok);
    chk(ok, "ADDR1 still answered with ADDR0_EN = 0");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own1, 0), e_data(8'h02), e_stop()}, "ADDR1 with ADDR0_EN = 0");
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, own0});
    // TGT_EN = 0 / EN = 0
    wr(R_CTRL, 32'h1);
    nack_case(own0, 0, "TGT_EN = 0");
    nack_case(own0, 1, "TGT_EN = 0 (read)");
    wr(R_CTRL, 32'h0);
    nack_case(own0, 0, "EN = 0");
    wr(R_CTRL, 32'h5);
    wait_dut_idle();
    ext_write(own0, '{8'h77}, ok);
    chk(ok, "own address acknowledged again");
    drain_trx(tq);
    chk_rd(R_FIFO_STAT, 0, '1, "TRX empty at the end");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_addr_filter_vseq

// ---------------------------------------------------------------- VP-TGT-04: several transactions queued, TRX_THR (6.10)
class i2c_tgt_trx_queue_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_trx_queue_vseq)
  function new(string name = "i2c_tgt_trx_queue_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] d2[$];
    bit [9:0] q[$];
    bit ok;
    int unsigned nt = depth / 4;
    wr(R_FIFO_CTRL, (depth / 2) << 24);
    wr(R_INTR_EN, 32'h100);
    for (int t = 0; t < nt; t++) begin
      rnd_bytes(2, d2);
      ext_write(t[0] ? own1 : own0, d2, ok);
      chk(ok, $sformatf("queued transaction %0d", t));
    end
    wait_cycles(3);
    chk(side.irq_tgt === (nt * 4 > depth / 2), $sformatf("irq_tgt_o by TRX_THR with %0d entries, threshold %0d", nt * 4, depth / 2));
    drain_trx(q);
    chk(q.size() == nt * 4, $sformatf("%0d TRX entries for %0d transactions", q.size(), nt));
    for (int t = 0; t < nt; t++)
      chk(q[4 * t][9:8] == 2'b01 && q[4 * t + 3] == e_stop(), $sformatf("transaction %0d framed by START / STOP entries", t));
    wr(R_INTR_EN, 0);
    wr(R_FIFO_CTRL, 0);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_trx_queue_vseq

// ---------------------------------------------------------------- VP-TGT-05: target transmitter, preloaded TTX (4.5.4, 6.11)
class i2c_tgt_read_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_read_vseq)
  function new(string name = "i2c_tgt_read_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] d[$], q[$];
    bit [9:0] tq[$];
    bit ok;
    int unsigned n;
    // TTX preloaded: bytes in order, controller NACKs the last -> TTX_DONE; TRX {01, A R}, {10, 00}
    w1c(EVENT_MASK);
    n = (depth < 4) ? depth : 4;
    rnd_bytes(n, d);
    load_ttx(d);
    ext_read(own0, n, q, ok);
    chk(ok && q == d, "read of a preloaded TTX");
    chk_ev(I_TTX_DONE, 1, "TTX_DONE:");
    chk_ev(I_TX_REQ, 0, "no TX_REQ with a preloaded TTX:");
    chk_rd(R_TSTAT, {21'h0, 1'b0, 1'b0, 1'b0, 1'b1, own0}, '1, "TSTAT RW = 1");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own0, 1), e_stop()}, "read transaction");
    w1c(EVENT_MASK);
    // controller reads less than loaded: leftover stays in TTX (not flushed automatically, 6.11 step 5) and is
    // served first by the next read (corner case); then TTX_FLUSH
    n = (depth < 5) ? depth : 5;
    rnd_bytes(n, d);
    load_ttx(d);
    ext_read(own0, 2, q, ok);
    chk(q[0] == d[0] && q[1] == d[1], "partial read");
    chk_ev(I_TTX_DONE, 1, "TTX_DONE after partial read:");
    chk_rd(R_FIFO_STAT, (n - 2) << 16, 32'h3F << 16, "TTX leftover not flushed automatically");
    if (n > 2) begin
      ext_read(own1, 1, q, ok);
      chk(ok && q[0] == d[2], "the next read is served from the leftover");
    end
    wr(R_CTRL, 32'h0000_0400 | 32'h5);                       // TTX_FLUSH (EN | TGT_EN kept)
    chk_rd(R_FIFO_STAT, 0, 32'h3F << 16, "TTX_FLUSH");
    drain_trx(tq);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_read_vseq

// ---------------------------------------------------------------- VP-TGT-06: TTX empty -> stretch + TX_REQ (8.5)
class i2c_tgt_tx_req_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_tx_req_vseq)
  function new(string name = "i2c_tgt_tx_req_vseq"); super.new(name); endfunction

  rand int unsigned n = 3;
  rand int unsigned sw_delay_us = 40;  // software reaction time
  constraint c_def { soft n == 3; n inside {[1:16]}; soft sw_delay_us == 40; sw_delay_us inside {[1:200]}; }

  task main();
    bit [7:0] d[$], q[$];
    bit [9:0] tq[$];
    bit ok;
    p_sequencer.side.clear_seen();
    w1c(EVENT_MASK);
    rnd_bytes(n, d);
    if (n > depth) n = depth;
    while (d.size() > n) void'(d.pop_back());
    fork
      ext_read(own0, n, q, ok);
      begin
        wait_ev(I_TX_REQ, 2ms, 0, "TX_REQ");
        chk_rd(R_STATUS, 32'h0000_0039, 32'h0000_0039, "STATUS: TGT_ACTIVE, TGT_RW, TGT_HOLD while waiting for TTX");
        w1c(32'h1 << I_TX_REQ);
        #(sw_delay_us * 1us);
        chk(bus.scl === 1'b0, "target holds SCL while TTX is empty");
        chk_ev(I_TX_REQ, 0, "no second TX_REQ after the W1C (6.12.2)");
        load_ttx(d);
      end
    join
    chk(ok && q == d, "read after TX_REQ stretch");
    chk(p_sequencer.side.seen_tgt_hold, "debug_tgt_hold_o seen");
    drain_trx(tq);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_tx_req_vseq

// ---------------------------------------------------------------- VP-TGT-07: read longer than TTX, refilled on TX_REQ;
//                                                                  register read flow of 6.11 (S W ptr Sr R ...)
class i2c_tgt_serviced_read_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_serviced_read_vseq)
  function new(string name = "i2c_tgt_serviced_read_vseq"); super.new(name); endfunction

  rand int unsigned n = 16;
  rand bit          reg_read;          // 1: S W ptr Sr R (6.11), 0: plain read
  rand bit          use_addr1;
  constraint c_def { n inside {[1:16]}; soft n == 16; }

  task main();
    bit [7:0] d[$], q[$];
    bit [9:0] rx[$];
    bit [6:0] a = use_addr1 ? own1 : own0;
    bit [7:0] p = rnd8();
    bit ok;
    rnd_bytes(n, d);
    ext_read_srv(a, d, n, q, ok, rx, reg_read ? int'(p) : -1);
    chk(ok && q == d, $sformatf("%s of %0d bytes served on TX_REQ", reg_read ? "register read" : "read", n));
    if (reg_read) chk_trx(rx, '{e_start(a, 0), e_data(p), e_rstart(a, 1), e_stop()}, "register read TRX");
    else          chk_trx(rx, '{e_start(a, 1), e_stop()}, "read TRX");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_serviced_read_vseq

// ---------------------------------------------------------------- VP-TGT-08: byte-level stretching on a full TRX (8.4)
class i2c_tgt_stretch_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_stretch_vseq)
  function new(string name = "i2c_tgt_stretch_vseq"); super.new(name); endfunction

  rand int unsigned sw_delay_us = 100;
  constraint c_def { soft sw_delay_us == 100; sw_delay_us inside {[20:300]}; }

  task main();
    bit [7:0] d[$];
    bit ok, done;
    int unsigned nmsg = depth / 18 + 1, got = 0;
    p_sequencer.side.clear_seen();
    fork
      begin
        for (int m = 0; m < nmsg; m++) begin
          rnd_bytes(16, d);
          ext_write(own0, d, ok);
          chk(ok, $sformatf("message %0d fully acknowledged (stretch, no loss)", m));
        end
        done = 1;
      end
      begin
        wait (side.d_trx_lvl >= depth - 1);
        #(sw_delay_us * 1us);                                // slow software: the target must hold SCL
        chk(p_sequencer.side.seen_tgt_hold && side.d_tgt_hold === 1'b1, "target stretches while TRX lacks 2 free entries");
        chk_rd(R_STATUS, 32'h0000_0020, 32'h0000_0020, "STATUS.TGT_HOLD");
        chk(bus.scl === 1'b0, "SCL held low by the target");
        while (!done || side.d_trx_lvl != 0) begin
          bit [31:0] v;
          if (side.d_trx_lvl != 0) begin rd(R_TRXDATA, v); got++; end
          #(15us);
        end
      end
    join
    chk(got == nmsg * 18, $sformatf("%0d TRX entries read, %0d expected", got, nmsg * 18));
    chk_ev(I_TRX_OVF, 0, "no TRX_OVF with stretching:");
  endtask
endclass : i2c_tgt_stretch_vseq

// ---------------------------------------------------------------- VP-TGT-09: NOSTRETCH receive (4.5.3)
class i2c_tgt_nostretch_rx_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_nostretch_rx_vseq)
  function new(string name = "i2c_tgt_nostretch_rx_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] d[$];
    bit [9:0] tq[$];
    bit ok;
    int unsigned lvl;
    p_sequencer.side.clear_seen();
    w1c(EVENT_MASK);
    // data byte NACKed when TRX has < 2 free entries, TRX_OVF
    ok = 1;
    for (int m = 0; m < 4 && ok; m++) begin
      rnd_bytes(16, d);
      ext_write(own0, d, ok);
    end
    chk(!ok, "a data byte is NACKed once TRX is full");
    chk_ev(I_TRX_OVF, 1, "TRX_OVF:");
    chk_rd(R_FIFO_STAT, depth << 24, 32'h3F << 24, "TRX full (STOP entry used the last place)");
    drain_trx(tq);
    w1c(EVENT_MASK);
    // address NACKed when TRX has < 2 free entries
    lvl = 0;
    while (depth - 1 - lvl >= 3) begin
      int unsigned nd = depth - 1 - lvl - 2;
      if (nd > 16) nd = 16;
      rnd_bytes(nd, d);
      ext_write(own0, d, ok);
      chk(ok, "fill TRX");
      lvl += nd + 2;
    end
    chk_rd(R_FIFO_STAT, (depth - 1) << 24, 32'h3F << 24, "TRX at FIFO_DEPTH - 1");
    ext_write(own0, '{8'h01}, ok);
    chk(!ok, "address NACKed with 1 free entry");
    chk_ev(I_TRX_OVF, 1, "TRX_OVF on address:");
    chk_ev(I_TADDR, 1, "TADDR of the fill transactions only:");
    chk_rd(R_FIFO_STAT, (depth - 1) << 24, 32'h3F << 24, "TRX unchanged");
    drain_trx(tq);
    chk(!p_sequencer.side.seen_tgt_hold, "the target never stretched SCL with NOSTRETCH");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_nostretch_rx_vseq

// ---------------------------------------------------------------- VP-TGT-09: NOSTRETCH transmit, TTX underflow (4.5.5)
class i2c_tgt_nostretch_tx_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_nostretch_tx_vseq)
  function new(string name = "i2c_tgt_nostretch_tx_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] q[$], d[$];
    bit [9:0] tq[$];
    bit ok;
    int unsigned k;
    p_sequencer.side.clear_seen();
    w1c(EVENT_MASK);
    ext_read(own0, 3, q, ok);
    chk(ok && q.size() == 3 && q[0] == 8'hFF && q[1] == 8'hFF && q[2] == 8'hFF, "empty TTX sends 0xFF");
    chk_ev(I_TTX_UNF, 1, "TTX_UNF:");
    chk_ev(I_TX_REQ, 0, "no TX_REQ with NOSTRETCH:");
    w1c(EVENT_MASK);
    k = $urandom_range(1, (depth < 4) ? depth - 1 : 3);
    rnd_bytes(k, d);
    load_ttx(d);
    ext_read(own0, k + 2, q, ok);
    for (int i = 0; i < k; i++) chk(q[i] == d[i], $sformatf("loaded byte %0d before the underflow", i));
    chk(q[k] == 8'hFF && q[k + 1] == 8'hFF, "TTX underflow after the loaded bytes");
    chk_ev(I_TTX_UNF, 1, "TTX_UNF after the loaded bytes:");
    chk(!p_sequencer.side.seen_tgt_hold, "the target never stretched SCL with NOSTRETCH");
    drain_trx(tq);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_nostretch_tx_vseq

// ---------------------------------------------------------------- VP-TGT-10: TNACK (5.2.11)
class i2c_tgt_tnack_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_tnack_vseq)
  function new(string name = "i2c_tgt_tnack_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] q[$];
    bit [9:0] tq[$];
    bit ok;
    bit [31:0] tcfg;
    rd(R_TCFG, tcfg);
    reg_wr(ral.TCFG, tcfg | 32'h4);
    w1c(EVENT_MASK);
    ext_write(own0, '{8'h01, 8'h02}, ok);
    chk(!ok && p_sequencer.sb.last_tgt_msg.addr_ack, "TNACK: address ACKed, data NACKed");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own0, 0), e_stop()}, "TNACK: no data entry");
    chk_ev(I_TADDR, 1, "TNACK TADDR:"); chk_ev(I_TSTOP, 1, "TNACK TSTOP:");
    w1c(EVENT_MASK);
    load_ttx('{8'h81, 8'h82});                               // transmit is not affected
    ext_read(own0, 2, q, ok);
    chk(ok && q[0] == 8'h81 && q[1] == 8'h82, "TNACK does not affect target transmit");
    drain_trx(tq);
    reg_wr(ral.TCFG, tcfg & ~32'h4);
    ext_write(own0, '{8'h05}, ok);
    chk(ok, "TNACK = 0: data ACKed again");
    drain_trx(tq);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_tnack_vseq

// ---------------------------------------------------------------- VP-TGT-11: general call (4.5.1, 5.2.11)
class i2c_tgt_gcall_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_gcall_vseq)
  function new(string name = "i2c_tgt_gcall_vseq"); super.new(name); endfunction

  task main();
    bit [9:0] tq[$], q2[$];
    bit ok;
    w1c(EVENT_MASK);
    reg_wr(ral.TCFG, 0);
    ext_write(7'h00, '{8'h06}, ok);
    chk(!ok, "general call with GC_EN = 0 not acknowledged");
    reg_wr(ral.TCFG, 1);
    ext_write(7'h00, '{8'h06}, ok);
    chk(ok, "general call acknowledged with GC_EN = 1");
    chk_ev(I_GC_SWRST, 1, "GC_SWRST:");
    chk_ev(I_TADDR, 1, "TADDR for a general call:");
    chk_rd(R_TSTAT, 32'h0000_0100, '1, "TSTAT.GC");
    drain_trx(tq);
    chk_trx(tq, '{e_start(7'h00, 0), e_data(8'h06), e_stop()}, "general call 0x06");
    w1c(EVENT_MASK);
    ext_write(7'h00, '{8'h04, 8'h55}, ok);
    chk(ok, "general call 0x04");
    chk_ev(I_GC_SWRST, 0, "no GC_SWRST for 0x04:");
    drain_trx(tq);
    chk_trx(tq, '{e_start(7'h00, 0), e_data(8'h04), e_data(8'h55), e_stop()}, "general call 0x04");
    w1c(EVENT_MASK);
    // hardware general call (second byte B = 1) is not supported (1.4): ACKed and pushed as plain data
    ext_write(7'h00, '{8'h0B, 8'h77}, ok);
    chk(ok, "hardware general call (B = 1) acknowledged as data");
    chk_ev(I_GC_SWRST, 0, "no GC_SWRST for a hardware general call:");
    drain_trx(tq);
    chk_trx(tq, '{e_start(7'h00, 0), e_data(8'h0B), e_data(8'h77), e_stop()}, "hardware general call 0x0B");
    w1c(EVENT_MASK);
    // corner: general call then Sr to the own address in the same transaction -> Sr tag 11
    ext_write_col(7'h00, '{8'h06}, ok, tq, 0);
    ext_write_col(own0, '{8'h3C}, ok, q2);
    foreach (q2[i]) tq.push_back(q2[i]);
    chk_trx(tq, '{e_start(7'h00, 0), e_data(8'h06), e_rstart(own0, 0), e_data(8'h3C), e_stop()}, "general call + Sr own address");
    chk_ev(I_GC_SWRST, 1, "GC_SWRST in the combined transaction:");
    w1c(EVENT_MASK);
    // the general call read direction (= START byte 0000_0001) is never acknowledged
    begin
      bit [7:0] q[$];
      ext_read(7'h00, 1, q, ok);
      chk(!ok, "0000_0001 (START byte / GC read) not acknowledged");
    end
    reg_wr(ral.TCFG, 0);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_gcall_vseq

// ---------------------------------------------------------------- VP-TGT-12: reserved addresses, Hs codes (4.5.1, 1.4)
class i2c_tgt_rsvd_addr_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_rsvd_addr_vseq)
  function new(string name = "i2c_tgt_rsvd_addr_vseq"); super.new(name); endfunction

  task nack_case(bit [6:0] a, bit rd_, string what);
    bit [7:0] q[$];
    bit ok;
    if (rd_) ext_read(a, 1, q, ok); else ext_write(a, '{8'h06}, ok);
    chk(!ok, $sformatf("%s (0x%02h %s) not acknowledged", what, a, rd_ ? "R" : "W"));
  endtask

  task main();
    bit ok;
    w1c(EVENT_MASK);
    for (int a = 1; a <= 3; a++) begin nack_case(a, 0, "CBUS / reserved"); nack_case(a, 1, "CBUS / reserved"); end
    for (int a = 8'h7C; a <= 8'h7F; a++) nack_case(a, 0, "1111_1xx (Device ID disabled)");
    // Hs controller codes: NACK, HS_DET, Hs-mode until STOP (4.3.4)
    p_sequencer.side.clear_seen();
    for (int a = 4; a <= 7; a++) begin
      nack_case(a, a[0], "Hs controller code");
      chk_ev(I_HS_DET, 1, $sformatf("HS_DET for code 0x%02h:", {a[6:0], a[0]}));
      w1c(32'h1 << I_HS_DET);
    end
    chk(p_sequencer.side.seen_hs, "hs_mode_o set after a controller code + NACK");
    chk(side.hs_mode === 1'b0, "hs_mode_o cleared by STOP");
    chk_rd(R_FIFO_STAT, 0, '1, "nothing pushed for reserved addresses");
    chk_ev(I_TADDR, 0, "no TADDR for reserved addresses:");
    // own address programmed inside the reserved ranges never matches
    reg_wr(ral.TADDR, {16'h0, 1'b1, 7'h7E, 1'b1, 7'h02});
    nack_case(7'h02, 0, "ADDR0 programmed in the reserved range");
    nack_case(7'h7E, 0, "ADDR1 programmed in the reserved range");
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, own0});
    ext_write(own0, '{8'h77}, ok);
    chk(ok, "own address acknowledged after the reserved addresses");
    begin bit [9:0] tq[$]; drain_trx(tq); end
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_rsvd_addr_vseq

// ---------------------------------------------------------------- VP-TGT-13: 10-bit addressing not supported (1.4)
//   every refused header is followed by a normal write with data (checks the DUT and the VIP monitor state)
class i2c_tgt_addr10_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_addr10_vseq)
  function new(string name = "i2c_tgt_addr10_vseq"); super.new(name); endfunction

  task main();
    bit [9:0] tq[$];
    bit [7:0] q[$];
    bit ok;
    for (int a = 8'h78; a <= 8'h7B; a++) begin
      if (a[0]) ext_read(a, 1, q, ok); else ext_write(a, '{8'h06}, ok);
      chk(!ok, $sformatf("10-bit header 0x%02h not acknowledged", a));
      ext_write(own0, '{8'hA0 + a[1:0], 8'h5A}, ok);
      chk(ok, "normal write right after a refused 10-bit header");
      drain_trx(tq);
      chk_trx(tq, '{e_start(own0, 0), e_data(8'hA0 + a[1:0]), e_data(8'h5A), e_stop()}, "write after a 10-bit header");
    end
    reg_wr(ral.TADDR, {16'h0, 1'b1, 7'h7A, 1'b1, 7'h79});
    ext_write(7'h79, '{8'h01}, ok); chk(!ok, "ADDR0 programmed at a 10-bit header");
    ext_read(7'h7A, 1, q, ok);      chk(!ok, "ADDR1 programmed at a 10-bit header");
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, own0});
    p_sequencer.ext.addr10 = 1;
    ext_write(7'h55, '{8'h01}, ok);
    p_sequencer.ext.addr10 = 0;
    chk(!ok, "10-bit address not acknowledged");
    chk_rd(R_FIFO_STAT, 0, '1, "nothing pushed for a 10-bit header");
    ext_write(own1, '{8'h99}, ok);
    chk(ok, "normal write after the 10-bit address");
    drain_trx(tq);
    chk_trx(tq, '{e_start(own1, 0), e_data(8'h99), e_stop()}, "write after the 10-bit address");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_addr10_vseq

// ---------------------------------------------------------------- VP-TGT-14: Device ID (4.5.6, 6.9)
class i2c_tgt_devid_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_devid_vseq)
  function new(string name = "i2c_tgt_devid_vseq"); super.new(name); endfunction

  rand bit [23:0]   id = 24'hA5C35A;
  rand int unsigned n_wrap = 7;           // bytes of the wrap-around read
  constraint c_def { n_wrap inside {[4:12]}; }

  task main();
    bit [7:0]  q[$];
    bit [9:0]  tq[$];
    bit ok;
    reg_wr(ral.TCFG, 32'h8);                                  // DEVID_EN
    reg_wr(ral.DEVID, id);
    // S F8 A {ADDR0, x} A Sr F9 A ID2 A ID1 A ID0 NA P
    ext_write(7'h7C, '{{own0, 1'b0}}, ok, 0);
    chk(ok, "Device ID write + own address acknowledged");
    chk_rd(R_TSTAT, 32'h400, 32'h400, "TSTAT.DEVID_ACTIVE while armed");
    ext_read(7'h7C, 3, q, ok);
    chk(ok && q.size() == 3 && {q[0], q[1], q[2]} == id, $sformatf("Device ID read 0x%02h%02h%02h, DEVID 0x%06h", q[0], q[1], q[2], id));
    chk_rd(R_TSTAT, 0, 32'h400, "TSTAT.DEVID_ACTIVE cleared by STOP");
    // wrap-around, ADDR1 with R/W bit = 1 (LSB don't care)
    ext_write(7'h7C, '{{own1, 1'b1}}, ok, 0);
    chk(ok, "Device ID with ADDR1");
    ext_read(7'h7C, n_wrap, q, ok);
    begin
      bit good = (q.size() == n_wrap);
      foreach (q[i]) if (q[i] != id[23 - 8 * (i % 3) -: 8]) good = 0;
      chk(good, $sformatf("Device ID wraps after 3 bytes (%0d bytes read)", n_wrap));
    end
    // not armed / wrong address / disabled
    ext_read(7'h7C, 1, q, ok);
    chk(!ok, "0xF9 without the 0xF8 sequence is NACKed");
    ext_write(7'h7C, '{{7'h44, 1'b0}}, ok);
    chk(!ok, "Device ID sequence for a foreign address: second byte NACKed");
    // after arming, a Sr with a normal address disarms and is handled normally (tag 11)
    ext_write(7'h7C, '{{own0, 1'b0}}, ok, 0);
    ext_write(own0, '{8'h5E}, ok);
    chk(ok, "normal address after the Device ID write");
    drain_trx(tq);
    chk_trx(tq, '{e_rstart(own0, 0), e_data(8'h5E), e_stop()}, "Sr after 0xF8 sequence");
    reg_wr(ral.TCFG, 0);
    ext_write(7'h7C, '{{own0, 1'b0}}, ok);
    chk(!ok, "DEVID_EN = 0: 0xF8 NACKed");
    chk_ev(I_TX_REQ, 0, "Device ID never uses TTX:");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_devid_vseq

// ---------------------------------------------------------------- VP-SYS-01: controller + target in the same IP (4.6)
class i2c_dual_role_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_dual_role_vseq)
  function new(string name = "i2c_dual_role_vseq"); super.new(name); endfunction

  rand int unsigned n_iter = 4;
  constraint c_def { soft n_iter == 4; n_iter inside {[1:20]}; }

  task main();
    bit [7:0] d[$], d2[$], q[$];
    bit [9:0] tq[$];
    bit ok;
    for (int it = 0; it < n_iter; it++) begin
      rnd_bytes($urandom_range(1, (depth - 2 < 6) ? depth - 2 : 6), d);    // the transaction fits in TRX
      rnd_bytes($urandom_range(1, 6), d2);
      d2.push_front(8'h00);
      // the DUT controller gets its commands while the external controller owns the bus: it waits for bus_free
      fork
        ext_write(own0, d, ok);
        begin
          wait (side.d_bus_busy === 1'b1);
          #($urandom_range(2, 40) * 1us);
          ctrl_write(cfg.slv_addr[it % 2], d2);
        end
      join
      chk(ok, "external write while the DUT controller waits");
      drain_trx(tq);
      chk(tq.size() == d.size() + 2, "target entries");
      // and the other way round: the external controller waits for the DUT controller
      load_ttx('{8'hA0 + it, 8'hB0 + it});
      fork
        ctrl_write(cfg.slv_addr[(it + 1) % 2], d2);
        begin
          #(5us);
          ext_read(own1, 2, q, ok);
          chk(ok && q[0] == 8'hA0 + it && q[1] == 8'hB0 + it, "external read after the DUT transfer");
        end
      join
      drain_trx(tq);
      w1c(EVENT_MASK);
    end
    chk_ev(I_ARB_LOST, 0, "no arbitration loss:");
  endtask
endclass : i2c_dual_role_vseq

// ---------------------------------------------------------------- VP-TGT-16: address ACKed while TRX lacks 2 free entries
//   (stretch mode): the target holds SCL after the address ACK until the software reads TRX (4.5.3, 4.5.4)
class i2c_tgt_addr_hold_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_addr_hold_vseq)
  function new(string name = "i2c_tgt_addr_hold_vseq"); super.new(name); endfunction

  rand int unsigned sw_delay_us;
  rand bit          is_read;           // 1: the held address is a read (then TX_REQ once the entry is pushed)
  constraint c_def { sw_delay_us inside {[20:100]}; }

  task main();
    i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
    bit [7:0] d[$], q[$];
    bit [9:0] tq[$], exp[$];
    bit ok;
    int unsigned lvl = 0;
    // fill TRX to FIFO_DEPTH - 1 without any stretching
    while (depth - 1 - lvl >= 3) begin
      int unsigned nd = depth - 1 - lvl - 2;
      if (nd > 16) nd = 16;
      rnd_bytes(nd, d);
      ext_write(own0, d, ok);
      chk(ok, "fill TRX");
      lvl += nd + 2;
    end
    chk_rd(R_FIFO_STAT, (depth - 1) << 24, 32'h3F << 24, "TRX at FIFO_DEPTH - 1");
    p_sequencer.side.clear_seen();
    w1c(EVENT_MASK);
    rnd_bytes(2, d);
    if (is_read) begin                                       // read: address held, then TX_REQ, then data
      fork
        ext_read(own1, 2, q, ok);
        begin
          wait (side.d_tgt_hold === 1'b1);
          #(sw_delay_us * 1us);
          chk_ev(I_TX_REQ, 0, "no TX_REQ while the address entry is held:");
          repeat (depth - 1) begin bit [31:0] v; rd(R_TRXDATA, v); end
          wait_ev(I_TX_REQ, 1ms, 1, "TX_REQ after the held read address was pushed");
          load_ttx(d);
        end
      join
      chk(ok && q == d, "read completed after the address hold and TX_REQ");
      drain_trx(tq);
      chk_trx(tq, '{e_start(own1, 1), e_stop()}, "read after the address hold");
      w1c(EVENT_MASK);
      return;
    end
    fork
      begin ext_write(own1, d, ok); col.stop_req = 1; end
      begin
        wait (side.d_tgt_hold === 1'b1);
        chk_rd(R_STATUS, 32'h0000_0028, 32'h0000_0038, "STATUS: TGT_ACTIVE, TGT_HOLD (address waiting for room)");
        #(sw_delay_us * 1us);
        chk(bus.scl === 1'b0, "SCL held while the address entry waits for TRX room");
        chk_rd(R_FIFO_STAT, (depth - 1) << 24, 32'h3F << 24, "TRX unchanged while holding");
        repeat (depth - 1) begin bit [31:0] v; rd(R_TRXDATA, v); tq.push_back(v[9:0]); end   // software frees TRX
        chk(tq[0][9:8] == 2'b01 && tq[depth - 2] == e_stop(), "old entries read (fill transactions)");
        run_seq(col);
      end
    join
    chk(ok, "write completed after the hold");
    exp = '{e_start(own1, 0), e_data(d[0]), e_data(d[1]), e_stop()};
    chk_trx(col.q, exp, "transaction after the address hold");
    chk_ev(I_TRX_OVF, 0, "no TRX_OVF with stretching:");
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_addr_hold_vseq

// ---------------------------------------------------------------- VP-TGT-17: combined format (read followed by Sr)
//   S A R .. NA Sr ...: TRX tags (4.5.2), TTX order, Device ID after a read, read after a Device ID arming
class i2c_tgt_combined_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_combined_vseq)
  function new(string name = "i2c_tgt_combined_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_trx_collect_vseq col;
    bit [7:0] d[$], q[$], w[$];
    bit [9:0] tq[$];
    bit ok, ok2;
    int unsigned n = (depth < 4) ? depth : 3;
    // ---- S ADDR0 R d.. NA Sr ADDR1 W w P
    rnd_bytes(n, d);
    load_ttx(d);
    rnd_bytes(2, w);
    col = i2c_ip_trx_collect_vseq::type_id::create("col");
    fork
      begin
        ext_read_nostop(own0, n, q, ok);
        ext_write(own1, w, ok2);
        col.stop_req = 1;
      end
      run_seq(col);
    join
    chk(ok && ok2 && q == d, "read then Sr write: data");
    chk_trx(col.q, '{e_start(own0, 1), e_rstart(own1, 0), e_data(w[0]), e_data(w[1]), e_stop()}, "read + Sr write");
    chk_ev(I_TTX_DONE, 1, "TTX_DONE at the NACK of the read:");
    w1c(EVENT_MASK);
    // ---- S ADDR1 R d NA Sr ADDR0 R d.. NA P (both reads served from TTX in order)
    rnd_bytes(n, d);
    load_ttx(d);
    ext_read_nostop(own1, 1, q, ok);
    chk(ok && q[0] == d[0], "first read of the combined transaction");
    ext_read(own0, n - 1, q, ok);                            // n >= 3 (FIFO_DEPTH >= 4)
    for (int i = 0; i < n - 1; i++) chk(q[i] == d[i + 1], $sformatf("second read byte %0d from TTX in order", i));
    drain_trx(tq);
    chk_trx(tq, '{e_start(own1, 1), e_rstart(own0, 1), e_stop()}, "read + Sr read");
    chk_rd(R_FIFO_STAT, 0, 32'h3F << 16, "TTX consumed by the two reads");
    w1c(EVENT_MASK);
    // ---- Device ID armed, then Sr to the own address for a read (Sr disarms, handled normally, tag 11)
    reg_wr(ral.TCFG, 32'h8);
    load_ttx('{8'h5A});
    ext_write(7'h7C, '{{own0, 1'b0}}, ok, 0);
    chk(ok, "Device ID arming");
    ext_read(own0, 1, q, ok);
    chk(ok && q[0] == 8'h5A, "read after Device ID arming");
    drain_trx(tq);
    chk_trx(tq, '{e_rstart(own0, 1), e_stop()}, "read after Device ID arming");
    // ---- own read, then Sr Device ID sequence (0xF8 own, Sr 0xF9 R x3)
    begin
      bit [31:0] id;
      rd(R_DEVID, id);
      load_ttx('{8'hC3});
      ext_read_nostop(own0, 1, q, ok);
      chk(ok && q[0] == 8'hC3, "own read before the Device ID sequence");
      ext_write(7'h7C, '{{own0, 1'b0}}, ok, 0);
      chk(ok, "Device ID arming after a read in the same transaction");
      ext_read(7'h7C, 3, q, ok);
      chk(ok && {q[0], q[1], q[2]} == id[23:0], $sformatf("Device ID read after an own read (0x%02h%02h%02h)", q[0], q[1], q[2]));
      drain_trx(tq);
      chk(tq.size() >= 1 && tq[0] == e_start(own0, 1), "address entry of the own read");
    end
    reg_wr(ral.TCFG, 0);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_tgt_combined_vseq

// ---------------------------------------------------------------- VP-TGT-18: Device ID combined formats (4.5.6 D1..D11)
//   The external controller is the bit-bang agent (the VIP master cannot send an address without data byte).
class i2c_tgt_devid_corner_vseq extends i2c_ip_tgt_base_vseq;
  `uvm_object_utils(i2c_tgt_devid_corner_vseq)
  function new(string name = "i2c_tgt_devid_corner_vseq"); super.new(name); endfunction

  rand bit [23:0] id = 24'h3C_96A5;
  bit [6:0]       foreign;             // an address nobody answers (chosen in main, own addresses known there)

  protected bit ack, lost;

  task bb_s();  p_sequencer.bbs.start_c();  endtask
  task bb_sr(); p_sequencer.bbs.rstart_c(); endtask
  task bb_p();  p_sequencer.bbs.stop_c(); ext_settle(); endtask   // the DUT sees the STOP LAT cycles later
  task tx_chk(bit [7:0] b, bit exp_ack, string what);
    p_sequencer.bbs.tx(b, ack, lost);
    chk(ack == exp_ack, $sformatf("%s: byte 0x%02h %s, expected %s", what, b, ack ? "ACK" : "NACK", exp_ack ? "ACK" : "NACK"));
  endtask
  // read n Device ID bytes (ACK all but the last), first byte = DEVID byte index `from`
  task rd_id(int unsigned n, int unsigned from, string what);
    bit [7:0] v;
    for (int unsigned i = 0; i < n; i++) begin
      int unsigned k = (from + i) % 3;
      bit [7:0] e = (k == 0) ? id[23:16] : (k == 1) ? id[15:8] : id[7:0];
      p_sequencer.bbs.rx(v, i != n - 1);
      chk(v == e, $sformatf("%s: Device ID byte %0d = 0x%02h, expected 0x%02h", what, i, v, e));
    end
  endtask

  task main();
    bit [9:0] tq[$];
    bit [7:0] d = rnd8();
    do foreign = 7'($urandom_range(7'h08, 7'h77));
    while (foreign == own0 || foreign == own1 || cfg.is_vip_slave(foreign));
    reg_wr(ral.DEVID, id);
    reg_wr(ral.TCFG, 32'h0);
    w1c(EVENT_MASK);
    quiet();
    // D2: DEVID_EN = 0
    bb_s(); tx_chk(8'hF8, 0, "D2 0xF8 with DEVID_EN = 0"); bb_p();
    reg_wr(ral.TCFG, 32'h8);
    // D6: S F8 Sr F9 (no target address byte)
    bb_s(); tx_chk(8'hF8, 1, "D6 0xF8"); bb_sr(); tx_chk(8'hF9, 0, "D6 0xF9 without the target address byte"); bb_p();
    // D9: 0xF9 as the first byte
    bb_s(); tx_chk(8'hF9, 0, "D9 0xF9 not armed"); bb_p();
    // D1 + D4: a repeated 0xF8 cancels the arming, a foreign address does not re-arm
    bb_s(); tx_chk(8'hF8, 1, "D1 0xF8"); tx_chk({own0, 1'b0}, 1, "D3 ADDR0");
    chk_rd(R_TSTAT, 32'h400, 32'h400, "D3 TSTAT.DEVID_ACTIVE while armed");
    bb_sr(); tx_chk(8'hF8, 1, "D1 repeated 0xF8"); tx_chk({foreign, 1'b0}, 0, "D4 foreign address");
    chk_rd(R_TSTAT, 0, 32'h400, "D1 TSTAT.DEVID_ACTIVE cleared by the repeated 0xF8");
    bb_sr(); tx_chk(8'hF9, 0, "D9 0xF9 after the failed re-arming"); bb_p();
    // D4 then D1 + D3 + D7: a failed sequence followed by a good one (ADDR1, R/W bit don't care)
    bb_s(); tx_chk(8'hF8, 1, "D1 0xF8"); tx_chk({foreign, 1'b1}, 0, "D4 foreign address");
    bb_sr(); tx_chk(8'hF8, 1, "D1 0xF8 again"); tx_chk({own1, 1'b1}, 1, "D3 ADDR1 with R/W = 1");
    bb_sr(); tx_chk(8'hF9, 1, "D7 0xF9 armed"); rd_id(3, 0, "D7"); bb_p();
    chk_rd(R_TSTAT, 0, 32'h400, "D11 TSTAT.DEVID_ACTIVE cleared by STOP");
    // D5 + D8: extra byte after the address byte NACKed (still armed); a second 0xF9 starts at DEVID[23:16]
    bb_s(); tx_chk(8'hF8, 1, "D1 0xF8"); tx_chk({own0, 1'b0}, 1, "D3 ADDR0"); tx_chk(d, 0, "D5 third byte");
    bb_sr(); tx_chk(8'hF9, 1, "D5 0xF9 still armed"); rd_id(2, 0, "D7 two bytes");
    bb_sr(); tx_chk(8'hF9, 1, "D8 second 0xF9"); rd_id(4, 0, "D8 restart and wrap"); bb_p();
    // D7 / D9: DEVID_EN latched at the Sr
    bb_s(); tx_chk(8'hF8, 1, "D1 0xF8"); tx_chk({own1, 1'b0}, 1, "D3 ADDR1");
    reg_wr(ral.TCFG, 32'h0);                                 // the bus is held (SCL low) by the controller
    bb_sr(); tx_chk(8'hF9, 0, "D9 DEVID_EN = 0 latched at the Sr"); bb_p();
    reg_wr(ral.TCFG, 32'h8);
    // D3: an ADDR0 inside the reserved range never matches
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, 7'h05});
    bb_s(); tx_chk(8'hF8, 1, "D1 0xF8"); tx_chk({7'h05, 1'b0}, 0, "D3 reserved ADDR0 0x05"); bb_p();
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, own0});
    // Device ID transactions: no TRX entry, no TADDR / TSTOP
    ext_settle();
    chk_rd(R_FIFO_STAT, 0, 32'h3F << 24, "no TRX entry from Device ID transactions");
    chk_ev(I_TADDR, 0, "Device ID: no TADDR:");
    chk_ev(I_TSTOP, 0, "Device ID: no TSTOP:");
    // D9 / D10: a normal frame followed by Sr 0xF9 (not armed): NACK, STOP entry of the frame
    bb_s(); tx_chk({own0, 1'b0}, 1, "own address"); tx_chk(d, 1, "data");
    bb_sr(); tx_chk(8'hF9, 0, "D9 0xF9 after a normal frame"); bb_p();
    ext_settle();
    drain_trx(tq);
    chk_trx(tq, '{e_start(own0, 0), e_data(d), e_stop()}, "normal frame + Sr 0xF9");
    w1c(EVENT_MASK);
    wait_bus_free(20us);
    loud();
  endtask
endclass : i2c_tgt_devid_corner_vseq
