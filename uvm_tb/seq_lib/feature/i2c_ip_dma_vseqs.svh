// -----------------------------------------------------------------------------
// Layer 2 - DMA feature sequences (spec 3.4, 4.9, 5.2.22 - 5.2.24, 6.15)
//   The DMAC model answers dma_req with the APB element transfers; the handshake protocol (R8) is checked by the
//   SVA, the request-type sequences against the examples of 4.9.4, the data by the scoreboard (DMAC APB accesses go
//   through the same APB monitor as the CPU).
// -----------------------------------------------------------------------------

class i2c_ip_dma_base_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_dma_base_vseq)
  function new(string name = "i2c_ip_dma_base_vseq"); super.new(name); endfunction

  // start a DMA operation through the layer-1 sequence
  task dma_start(dma_sel_e sel, bit pfc, int unsigned count, int unsigned blk, output i2c_dma_op op,
                 input bit [31:0] tx[$] = '{}, input bit deny_single = 0, input bit okay_as_rsvd = 0, input bit prog_blk = 1);
    i2c_ip_dma_start_vseq s = i2c_ip_dma_start_vseq::type_id::create("dma_start");
    s.sel = sel; s.pfc = pfc; s.count = count; s.blk = blk; s.tx_data = tx;
    s.deny_single = deny_single; s.okay_as_rsvd = okay_as_rsvd; s.prog_blk = prog_blk;
    run_seq(s);
    op = s.op;
  endtask

  task dma_check(i2c_dma_op op, string what, string exp_log = "", realtime tmo = 5ms);
    i2c_ip_dma_check_vseq c = i2c_ip_dma_check_vseq::type_id::create("dma_check");
    c.op = op; c.what = what; c.exp_log = exp_log; c.tmo = tmo;
    run_seq(c);
  endtask
endclass : i2c_ip_dma_base_vseq

// ---------------------------------------------------------------- VP-DMA-01: SEL = CMD, DMAC flow control (6.15.1)
class i2c_dma_cmd_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_cmd_vseq)
  function new(string name = "i2c_dma_cmd_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    bit [7:0] d[$];
    rnd_bytes(4, d);
    dma_start(SEL_CMD, 0, 6, 4, o, '{c_start(7'h50, 0), c_wr(8'h80), c_wr(d[0]), c_wr(d[1]), c_wr(d[2]), c_wr(d[3], 1)});
    wait_op(o, 2ms, "SEL=CMD");
    if (depth >= 8) begin
      chk_dma_log(o, "DMA_BLOCK DMA_BLOCK", "SEL=CMD");
      chk(o.ack_log.size() == 2 && o.ack_log[0] == DMA_OKAY && o.ack_log[1] == DMA_LAST_OKAY, "SEL=CMD ack types OKAY, LAST_OKAY");
    end
    wait (side.irq === 1'b1);                                 // DMA_DONE -> irq_o (INTR_EN set by the test)
    chk_dma_done("SEL=CMD");
    wait_cmpl(0, "DMA write");
    for (int i = 0; i < 4; i++) chk(slv_mem(0, 8'h80 + i) == d[i], "data written through DMA commands");
  endtask
endclass : i2c_dma_cmd_vseq

// ---------------------------------------------------------------- VP-DMA-02: SEL = CRX (6.15.2, 4.9.4 examples)
class i2c_dma_crx_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_crx_vseq)
  function new(string name = "i2c_dma_crx_vseq"); super.new(name); endfunction

  rand bit pfc;

  task main();
    i2c_dma_op o;
    bit [12:0] c[$];
    if (!pfc) begin
      // PFC = 0, 6 bytes, DMAC DENYs SINGLE while >= 4 remain: SINGLE(DENY) x3 -> BLOCK -> SINGLE -> SINGLE(LAST_OKAY)
      dma_start(SEL_CRX, 0, 6, 4, o, '{}, 1);
      c = '{c_start(7'h50, 0), c_wr(8'h80), c_start(7'h50, 1)};
      for (int i = 0; i < 6; i++) c.push_back(c_rd(0, i == 5));
      push_cmds(c);
      wait_op(o, 5ms, "SEL=CRX PFC=0");
      chk_dma_log(o, "DMA_SINGLE(D) DMA_SINGLE(D) DMA_SINGLE(D) DMA_BLOCK DMA_SINGLE DMA_SINGLE", "SEL=CRX PFC=0");
      chk(o.ack_log[o.ack_log.size() - 1] == DMA_LAST_OKAY, "last ack LAST_OKAY");
      chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0000, "DMA_STAT.DENY after DENY answers");
      chk_dma_done("SEL=CRX PFC=0");
      wait_cmpl(0, "DMA read PFC=0");
      foreach (o.rx_data[i]) chk(o.rx_data[i][7:0] == slv_mem(0, 8'h80 + i), $sformatf("DMA read byte %0d", i));
    end else begin
      // PFC = 1, 12 bytes (byte 12 LAST): BLOCK -> BLOCK -> LAST_BLOCK
      dma_start(SEL_CRX, 1, 32, 4, o);
      c = '{c_start(7'h51, 1)};
      for (int i = 0; i < 12; i++) c.push_back(c_rd(0, i == 11));
      push_cmds(c);
      dma_check(o, "SEL=CRX PFC=1", "DMA_BLOCK DMA_BLOCK DMA_LAST_BLOCK");
      chk(o.rx_data.size() == 12 && o.rx_data[11][8] && !o.rx_data[10][8], "12 words, LAST only on the 12th (CRXDATA.LAST)");
      wait_cmpl(0, "DMA read PFC=1");
    end
  endtask
endclass : i2c_dma_crx_vseq

// ---------------------------------------------------------------- VP-DMA-03: SEL = TRX, PFC (6.15.3)
class i2c_dma_trx_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_trx_vseq)
  function new(string name = "i2c_dma_trx_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    bit [7:0] d[$];
    bit ok;
    int unsigned blk = (depth >= 8) ? 4 : 2;                 // PFC needs a block that fits below FIFO_DEPTH - 1
    // ADDR + 5 data + STOP -> BLOCK, SINGLE, SINGLE, LAST_SINGLE
    rnd_bytes(5, d);
    dma_start(SEL_TRX, 1, 32, blk, o);
    ext_write(own0, d, ok);
    dma_check(o, "SEL=TRX", (blk == 4) ? "DMA_BLOCK DMA_SINGLE DMA_SINGLE DMA_LAST_SINGLE" : "", 2ms);
    chk(o.rx_data.size() == 7 && o.rx_data[0][9:0] == e_start(own0, 0) && o.rx_data[6][9:0] == e_stop(),
        "one DMA operation = one transaction (address ... STOP)");
    // next transaction: ADDR + 2 data + STOP = 4 entries -> LAST_BLOCK (lpos = BLK)
    dma_start(SEL_TRX, 1, 32, blk, o, '{}, 0, 0, 0);
    ext_write(own1, '{8'h01, 8'h02}, ok);
    dma_check(o, "SEL=TRX second", (blk == 4) ? "DMA_LAST_BLOCK" : "", 2ms);
  endtask
endclass : i2c_dma_trx_vseq

// ---------------------------------------------------------------- VP-DMA-04: SEL = TTX (6.15.4)
class i2c_dma_ttx_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_ttx_vseq)
  function new(string name = "i2c_dma_ttx_vseq"); super.new(name); endfunction

  rand int unsigned n = 6;
  constraint c_def { soft n == 6; n inside {[1:16]}; }

  task main();
    i2c_dma_op o;
    bit [7:0] d[$], q[$];
    bit [31:0] tx[$];
    bit [9:0] tq[$];
    bit ok;
    int unsigned blk = (depth >= 8) ? 4 : 2;
    rnd_bytes(n, d);
    foreach (d[i]) tx.push_back(d[i]);
    dma_start(SEL_TTX, 0, n, blk, o, tx);
    fork
      wait_op(o, 5ms, "SEL=TTX");
      begin #(20us); ext_read(own0, n, q, ok); end
    join
    if (depth >= 8 && n == 6) chk_dma_log(o, "DMA_BLOCK DMA_BLOCK", "SEL=TTX");
    chk(ok && q == d, "external read of the DMA-loaded TTX bytes");
    chk_dma_done("SEL=TTX");
    drain_trx(tq);
  endtask
endclass : i2c_dma_ttx_vseq

// ---------------------------------------------------------------- VP-DMA-05: block size rules (5.2.23)
class i2c_dma_blk_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_blk_vseq)
  function new(string name = "i2c_dma_blk_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    bit [31:0] tx[$];
    // BLK = 0: SINGLE only (RX, PFC): SINGLE, SINGLE, LAST_SINGLE; reserved ack_type = OKAY, also for LAST_SINGLE
    dma_start(SEL_CRX, 1, 8, 0, o, '{}, 0, 1);
    push_cmds('{c_start(7'h50, 1), c_rd(), c_rd(), c_rd(0, 1)});
    dma_check(o, "BLK=0", "DMA_SINGLE DMA_SINGLE DMA_LAST_SINGLE", 2ms);
    wait_cmpl(0, "BLK=0 read");
    // BLK > FIFO_DEPTH is clamped to FIFO_DEPTH: one BLOCK of FIFO_DEPTH commands
    tx.push_back(c_start(7'h51, 0));
    tx.push_back(c_wr(8'h40));
    for (int i = 0; i < depth - 2; i++) tx.push_back(c_wr(i, i == depth - 3));
    wr(R_DMA_BLK, 63);
    dma_start(SEL_CMD, 0, depth, depth, o, tx, 0, 0, 0);
    wait_op(o, 2ms, "BLK clamp");
    chk_dma_log(o, "DMA_BLOCK", "BLK=63 clamped");
    chk(o.n_slverr == 0, "no CMD overflow");
    chk_dma_done("BLK clamp");
    wait_cmpl(0, "clamped block write");
  endtask
endclass : i2c_dma_blk_vseq

// ---------------------------------------------------------------- VP-DMA-06: request held, register lock, SWRST (4.9.2, 5.2.1)
class i2c_dma_req_hold_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_req_hold_vseq)
  function new(string name = "i2c_dma_req_hold_vseq"); super.new(name); endfunction

  task main();
    // a request is never withdrawn (EN = 0), SEL/PFC/BLK locked while pending, DENY releases it
    cfg.dma_cfg.lat_min = 300; cfg.dma_cfg.lat_max = 300;
    p_sequencer.dmac.stop_op();                               // DMAC channel stopped: it will DENY
    wr(R_DMA_BLK, 4);
    wr(R_DMA_CTRL, 32'h21);                                   // SEL = TTX (empty: avail = depth) -> request
    wait_cycles(30);
    chk(cfg.dma_cfg.vif.req === 1'b1, "request issued");
    chk_rd(R_DMA_STAT, 1, 1, "DMA_STAT.REQ = 1");
    wr(R_DMA_CTRL, 32'h12);                                   // EN = 0, SEL = CRX, PFC = 1: SEL/PFC ignored
    wr(R_DMA_BLK, 7);
    chk(cfg.dma_cfg.vif.req === 1'b1, "request kept after EN = 0");
    chk_rd(R_DMA_CTRL, 32'h20, '1, "SEL / PFC locked while a request is pending, EN written");
    chk_rd(R_DMA_BLK, 4, '1, "DMA_BLK locked while a request is pending");
    wait (cfg.dma_cfg.vif.req === 1'b0);
    wait_cycles(5);
    chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0001, "DMA_STAT.DENY, REQ = 0");
    wr(R_DMA_CTRL, 32'h12);
    chk_rd(R_DMA_CTRL, 32'h12, '1, "SEL / PFC writable again");
    wr(R_DMA_STAT, 32'h0001_0000);
    chk_rd(R_DMA_STAT, 0, '1, "DENY W1C");
    // SWRST does not withdraw a pending request (5.2.1)
    wr(R_DMA_CTRL, 32'h21);
    wait_cycles(30);
    wr(R_CTRL, 32'h8000_0000, 4'b1000);
    wait_cycles(3);
    chk(cfg.dma_cfg.vif.req === 1'b1, "request kept across SWRST");
    chk_rd(R_DMA_CTRL, 32'h20, '1, "SWRST cleared DMA_CTRL.EN");
    wait (cfg.dma_cfg.vif.req === 1'b0);
    cfg.dma_cfg.lat_min = 2; cfg.dma_cfg.lat_max = 6;
    p_sequencer.sb.resync();
    wr(R_DMA_STAT, 32'h0001_0100);
    wr(R_DMA_CTRL, 32'h0);
    wr(R_CTRL, 32'h3);                                        // SWRST cleared EN / CTRL_EN
    wait_dut_idle();
  endtask
endclass : i2c_dma_req_hold_vseq

// ---------------------------------------------------------------- VP-DMA-07: DENY rules (4.9.2, 4.9.5)
class i2c_dma_deny_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_deny_vseq)
  function new(string name = "i2c_dma_deny_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    int unsigned nr;
    // after DENY no new request until avail changes
    dma_start(SEL_CRX, 0, 8, 4, o, '{}, 1);                   // DENY SINGLE while >= 4 remain
    push_cmds('{c_start(7'h50, 1), c_rd(0, 1)});              // one byte in CRX
    wait_cmpl(0, "one byte read");
    #(20us);
    nr = p_sequencer.dmac.n_req;
    #(100us);
    chk(p_sequencer.dmac.n_req == nr, "no request repeated after DENY while CRX level is unchanged");
    push_cmds('{c_start(7'h50, 1), c_rd(0, 1)});              // level 1 -> 2: one new request
    wait_cmpl(0, "second byte read");
    #(20us);
    chk(p_sequencer.dmac.n_req == nr + 1, "one new request after the level changed");
    chk_dma_log(o, "DMA_SINGLE(D) DMA_SINGLE(D)", "DENY blocking");
    p_sequencer.dmac.stop_op();
    wr(R_DMA_CTRL, 32'h10);
    wait (cfg.dma_cfg.vif.req === 1'b0);
    wr(R_CTRL, 32'h0000_0203);                                // CRX_FLUSH
    wr(R_DMA_STAT, 32'h0001_0100);
    // the DMAC DENYs LAST_SINGLE / LAST_BLOCK of a stopped channel (request clear, 4.9.5)
    p_sequencer.dmac.stop_op();
    wr(R_DMA_BLK, 0);
    wr(R_DMA_CTRL, 32'h13);                                   // CRX, PFC, BLK = 0
    nr = p_sequencer.dmac.n_deny;
    push_cmds('{c_start(7'h50, 1), c_rd(0, 1)});              // one byte with LAST -> LAST_SINGLE
    wait_cmpl(0, "LAST_SINGLE denied");
    wait (cfg.dma_cfg.vif.req === 1'b0);
    chk(p_sequencer.dmac.n_deny == nr + 1, "LAST_SINGLE request answered with DENY");
    chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0101, "LAST_SINGLE DENYed: DENY, not DONE");
    wr(R_DMA_CTRL, 32'h12);
    wr(R_CTRL, 32'h0000_0203);
    wr(R_DMA_BLK, 4);
    wr(R_DMA_CTRL, 32'h13);
    nr = p_sequencer.dmac.n_deny;
    push_cmds('{c_start(7'h50, 1), c_rd(), c_rd(), c_rd(), c_rd(0, 1)});   // 4 bytes, LAST on the 4th -> LAST_BLOCK
    wait_cmpl(0, "LAST_BLOCK denied");
    wait (cfg.dma_cfg.vif.req === 1'b0);
    chk(p_sequencer.dmac.n_deny == nr + 1, "LAST_BLOCK request answered with DENY");
    chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0101, "LAST_BLOCK DENYed: DENY, not DONE");
    wr(R_DMA_CTRL, 32'h12);
    wr(R_CTRL, 32'h0000_0203);
    wr(R_DMA_STAT, 32'h0001_0100);
    wr(R_DMA_CTRL, 32'h0);
  endtask
endclass : i2c_dma_deny_vseq

// ---------------------------------------------------------------- VP-DMA-08: ack types (4.9.3)
class i2c_dma_ack_type_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_ack_type_vseq)
  function new(string name = "i2c_dma_ack_type_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    // PFC in the IP (DMA_CTRL.PFC = 1), DMAC counting the same end (op.pfc = 0): LAST_* answered with LAST_OKAY
    o = mk_op(SEL_CRX, 0, 1, 0);
    p_sequencer.dmac.start_op(o);
    wr(R_DMA_BLK, 0);
    wr(R_DMA_CTRL, 32'h13);
    push_cmds('{c_start(7'h50, 1), c_rd(0, 1)});
    dma_check(o, "LAST_SINGLE / LAST_OKAY", "DMA_LAST_SINGLE", 2ms);
    wait_cmpl(0, "LAST_SINGLE / LAST_OKAY read");
    o = mk_op(SEL_CRX, 0, 4, 4);
    p_sequencer.dmac.start_op(o);
    wr(R_DMA_BLK, 4);
    wr(R_DMA_CTRL, 32'h13);
    push_cmds('{c_start(7'h50, 1), c_rd(), c_rd(), c_rd(), c_rd(0, 1)});
    dma_check(o, "LAST_BLOCK / LAST_OKAY", "DMA_LAST_BLOCK", 2ms);
    wait_cmpl(0, "LAST_BLOCK / LAST_OKAY read");
    // reserved ack_type 2'b11 is treated as OKAY
    dma_start(SEL_CMD, 0, 6, 4, o, '{c_start(7'h50, 0), c_wr(8'h90), c_wr(8'h01), c_wr(8'h02), c_wr(8'h03), c_wr(8'h04, 1)}, 0, 1);
    wait_op(o, 2ms, "reserved ack_type");
    chk(o.ack_log[0] == DMA_ACK_RSVD, "first block answered with ack_type 2'b11");
    chk_dma_done("reserved ack_type");
    wait_cmpl(0, "reserved ack_type write");
  endtask
endclass : i2c_dma_ack_type_vseq

// ---------------------------------------------------------------- VP-DMA-09: controller abort with SEL = CMD (4.4.9, 6.13)
class i2c_dma_abort_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_abort_vseq)
  function new(string name = "i2c_dma_abort_vseq"); super.new(name); endfunction

  task main();
    i2c_dma_op o;
    dma_start(SEL_CMD, 0, 20, 0, o, '{c_start(7'h23, 0), c_wr(8'h01), c_wr(8'h02), c_wr(8'h03, 1)});
    wait_ev(I_NACK, 2ms, 1, "NACK of the DMA transfer");
    wait_cmpl(7'h01, "NACK abort with SEL = CMD");
    chk_rd(R_DMA_CTRL, 0, 32'h1, "DMA_CTRL.EN cleared by the abort");
    p_sequencer.dmac.stop_op();
    wait (cfg.dma_cfg.vif.req === 1'b0);
    wr(R_CTRL, 32'h0000_0103);                                // CMD_FLUSH (6.13)
    chk_rd(R_DMA_STAT, 0, 32'h1, "no pending request");
    wr(R_DMA_STAT, 32'h0001_0100);
    wr(R_DMA_CTRL, 32'h0);
    ctrl_write(7'h50, '{8'h00, 8'h01});
  endtask
endclass : i2c_dma_abort_vseq

// ---------------------------------------------------------------- VP-DMA-10 (random): random DMA operations
//   CMD write (DMAC flow control), CRX read (PFC), TRX receive (PFC), TTX transmit; random lengths / block sizes;
//   random end signalling of the DMAC (4-phase end handshake, terminal-count pulse, none) with END_EN = 0.
//   The data is checked against the VIP slave memory / the external controller; the request sequence by the SVA.
class i2c_dma_rand_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_rand_vseq)
  function new(string name = "i2c_dma_rand_vseq"); super.new(name); endfunction

  rand int unsigned n_iter = 12;
  constraint c_def { soft n_iter == 12; n_iter inside {[1:100]}; }

  task main();
    for (int it = 0; it < n_iter; it++) begin
      i2c_dma_op o;
      int unsigned kind = $urandom_range(3);
      int unsigned blk  = $urandom_range(0, depth);
      bit [7:0] d[$], q[$];
      bit [31:0] tx[$];
      bit ok;
      cfg.dma_cfg.end_mode = dma_end_mode_e'($urandom_range(2));
      case (kind)
        0: begin                                              // CMD: S A W ptr d.. P through DMA
          int unsigned n = $urandom_range(1, 2 * depth);
          bit [7:0] p = rnd8();
          rnd_bytes(n, d);
          tx.push_back(c_start(7'h50, 0)); tx.push_back(c_wr(p));
          foreach (d[i]) tx.push_back(c_wr(d[i], i == n - 1));
          dma_start(SEL_CMD, 0, tx.size(), blk, o, tx);
          dma_check(o, $sformatf("rand CMD %0d words BLK %0d", tx.size(), blk), "", 20ms);
          wait_cmpl(0, "rand DMA write");
          foreach (d[i]) chk(slv_mem(0, p + i) == d[i], $sformatf("rand DMA write byte %0d", i));
        end
        1: begin                                              // CRX, PFC: read n bytes, LAST ends the operation
          int unsigned n = $urandom_range(1, 2 * depth);
          bit [7:0] p = rnd8();
          bit [12:0] c[$];
          if (blk > depth - 1) blk = depth - 1;
          dma_start(SEL_CRX, 1, 64, blk, o);
          c = '{c_start(7'h51, 0), c_wr(p), c_start(7'h51, 1)};
          for (int i = 0; i < n; i++) c.push_back(c_rd(0, i == n - 1));
          push_cmds(c);
          dma_check(o, $sformatf("rand CRX %0d bytes BLK %0d", n, blk), "", 20ms);
          wait_cmpl(0, "rand DMA read");
          chk(o.rx_data.size() == n, $sformatf("rand CRX: %0d words for %0d bytes", o.rx_data.size(), n));
          foreach (o.rx_data[i]) chk(o.rx_data[i][7:0] == slv_mem(1, p + i) && o.rx_data[i][8] == (i == n - 1),
                                     $sformatf("rand CRX word %0d = %03h", i, o.rx_data[i]));
        end
        2: begin                                              // TRX, PFC: one transaction
          int unsigned n = $urandom_range(1, 16);
          if (blk > depth - 2) blk = depth - 2;
          rnd_bytes(n, d);
          dma_start(SEL_TRX, 1, 64, blk, o);
          ext_write($urandom_range(1) ? own0 : own1, d, ok);
          dma_check(o, $sformatf("rand TRX %0d bytes BLK %0d", n, blk), "", 20ms);
          chk(ok && o.rx_data.size() == n + 2, $sformatf("rand TRX: %0d words for %0d bytes", o.rx_data.size(), n));
          for (int i = 0; i < n && i + 1 < o.rx_data.size(); i++) chk(o.rx_data[i + 1][9:0] == e_data(d[i]), "rand TRX data word");
        end
        3: begin                                              // TTX: DMAC fills, the external controller reads
          int unsigned n = $urandom_range(1, (depth < 16) ? depth : 16);
          bit [9:0] tq[$];
          rnd_bytes(n, d);
          foreach (d[i]) tx.push_back(d[i]);
          dma_start(SEL_TTX, 0, n, blk, o, tx);
          fork
            wait_op(o, 5ms, "rand TTX");
            begin #(10us); ext_read($urandom_range(1) ? own0 : own1, n, q, ok); end
          join
          chk(ok && q == d, "rand TTX: external read of the DMA-loaded bytes");
          chk_dma_done("rand TTX");
          drain_trx(tq);
        end
      endcase
      w1c(EVENT_MASK);
    end
    cfg.dma_cfg.end_mode = END_HS;
    wr(R_DMA_STAT, 32'h0101_0100);
    wr(R_DMA_CTRL, 32'h0);
  endtask
endclass : i2c_dma_rand_vseq

// ---------------------------------------------------------------- VP-DMA-11: RX PFC with the LAST entry already in the FIFO
//   (channel enabled after the read): request types from the selection rule of 4.9.2 (lpos vs BLK)
class i2c_dma_pfc_late_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_pfc_late_vseq)
  function new(string name = "i2c_dma_pfc_late_vseq"); super.new(name); endfunction

  rand int unsigned n, blk;
  constraint c_def { n inside {[2:32]}; blk inside {[0:32]}; }

  // expected request sequence (4.9.2 pseudo-code, PFC = 1, the LAST entry at position lpos)
  function string exp_log(int unsigned lpos, int unsigned b);
    string s = "";
    while (lpos > 0) begin
      string t;
      int unsigned k;
      if (lpos == 1)                   begin t = "DMA_LAST_SINGLE"; k = 1; end
      else if (b != 0 && lpos == b)    begin t = "DMA_LAST_BLOCK";  k = b; end
      else if (b != 0 && lpos > b)     begin t = "DMA_BLOCK";       k = b; end
      else                             begin t = "DMA_SINGLE";      k = 1; end
      s = (s == "") ? t : {s, " ", t};
      lpos -= k;
    end
    return s;
  endfunction

  task main();
    i2c_dma_op o;
    bit [12:0] c[$];
    bit [7:0]  p = rnd8();
    if (n > depth) n = depth;
    if (blk > n) blk = n;
    c = '{c_start(7'h50, 0), c_wr(p), c_start(7'h50, 1)};
    for (int i = 0; i < n; i++) c.push_back(c_rd(0, i == n - 1));
    push_cmds(c);
    wait_cmpl(0, $sformatf("read of %0d bytes (DMA disabled)", n));
    chk_rd(R_FIFO_STAT, n << 8, 32'h3F << 8, "all bytes in CRX, LAST included");
    dma_start(SEL_CRX, 1, 64, blk, o);                        // enabled now: lpos = n
    dma_check(o, $sformatf("late PFC n=%0d BLK=%0d", n, blk), exp_log(n, blk), 2ms);
    chk(o.rx_data.size() == n, $sformatf("%0d words moved", o.rx_data.size()));
    foreach (o.rx_data[i]) chk(o.rx_data[i][7:0] == slv_mem(0, p + i) && o.rx_data[i][8] == (i == n - 1),
                               $sformatf("late PFC word %0d = %03h", i, o.rx_data[i]));
  endtask
endclass : i2c_dma_pfc_late_vseq

// ---------------------------------------------------------------- VP-DMA-12: end handshake from the DMAC (4.9.6, 5.2.22, 5.2.24)
//   dma_end_req_i / dma_end_ack_o (DMA-350 trigger output): acknowledged one cycle later (SVA_DMA_END_ACK), always
//   recorded in DMA_STAT.END, completes the channel only with DMA_CTRL.END_EN = 1 and EN = 1, never withdraws a request
class i2c_dma_end_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_end_vseq)
  function new(string name = "i2c_dma_end_vseq"); super.new(name); endfunction

  // DMA_DONE events since the start of the test (sideband event counter)
  function int unsigned n_done();
    return p_sequencer.side.ev_cnt[I_DMA_DONE];
  endfunction

  task main();
    i2c_dma_op o;
    bit [7:0] d[$];
    int unsigned ne, nd;
    int unsigned blk = (depth >= 8) ? 4 : 2;

    // (a) DMAC reporting its last transfer with LAST_OKAY (DMA-350), END_EN = 0: the 4-phase end handshake that
    //     follows is acknowledged and only recorded in DMA_STAT.END - one DMA_DONE per operation
    cfg.dma_cfg.end_mode = END_HS;
    cfg.dma_cfg.end_lat_min = 400; cfg.dma_cfg.end_lat_max = 400;
    ne = p_sequencer.dmac.n_end;
    rnd_bytes(3, d);
    dma_start(SEL_CMD, 0, 5, blk, o, '{c_start(7'h50, 0), c_wr(8'hA0), c_wr(d[0]), c_wr(d[1]), c_wr(d[2], 1)});
    chk_dma_done("END_EN=0, LAST_OKAY");                      // completed by LAST_OKAY; DONE / DMA_DONE cleared
    chk_rd(R_DMA_STAT, 0, 32'h0100_0000, "END_EN=0: no end event yet");
    nd = n_done();
    wait_op(o, 2ms, "END_EN=0");                              // end handshake finished
    chk(p_sequencer.dmac.n_end == ne + 1, "END_EN=0: end handshake acknowledged");
    chk_rd(R_DMA_STAT, 32'h0100_0000, 32'h0101_0103, "END_EN=0: DMA_STAT.END set, no new DONE, channel idle");
    chk(n_done() == nd, "END_EN=0: the end event creates no DMA_DONE");
    chk_ev(I_DMA_DONE, 0, "END_EN=0:");
    wr(R_DMA_STAT, 32'h0100_0000);
    chk_rd(R_DMA_STAT, 0, 32'h0100_0000, "DMA_STAT.END W1C");
    wait_cmpl(0, "END_EN=0 write");
    foreach (d[i]) chk(slv_mem(0, 8'hA0 + i) == d[i], "END_EN=0 data");
    // END_EN = 0 with the channel still enabled and a request pending (DMAC without DENY, stopped): end events
    // (4-phase, pulse) are only recorded - EN stays 1, the request stays pending until CLR
    cfg.dma_cfg.no_deny = 1;
    p_sequencer.dmac.stop_op();
    nd = n_done();
    wr(R_DMA_CTRL, 32'h21);                                   // SEL = TTX (empty): request, EN, END_EN = 0
    wait (cfg.dma_cfg.vif.req === 1'b1);
    p_sequencer.dmac.send_end(0);
    p_sequencer.dmac.send_end(1, 2);
    chk_rd(R_DMA_CTRL, 32'h21, '1, "END_EN=0: EN kept after end events");
    chk_rd(R_DMA_STAT, 32'h0100_0003, 32'h0101_0103, "END_EN=0: END recorded, no DONE, request still pending");
    chk(n_done() == nd, "END_EN=0, EN=1: no DMA_DONE");
    wr(R_DMA_CTRL, 32'h08);                                   // CLR, EN = 0
    wait_cycles(4);
    chk_rd(R_DMA_STAT, 32'h0100_0000, 32'h0100_0003, "request withdrawn");
    wr(R_DMA_STAT, 32'h0100_0000);
    cfg.dma_cfg.no_deny = 0;

    // (b) DMAC without LAST_OKAY (DMAC flow control, every ack OKAY), END_EN = 1, terminal-count pulse: the final
    //     OKAY does not complete the channel, the end pulse does (EN cleared, DONE, DMA_DONE, END)
    cfg.dma_cfg.end_mode = END_PULSE;
    nd = n_done();
    rnd_bytes(3, d);
    o = mk_op(SEL_CMD, 0, 5, blk);
    o.tx_data = '{c_start(7'h50, 0), c_wr(8'hB0), c_wr(d[0]), c_wr(d[1]), c_wr(d[2], 1)};
    o.no_last_okay = 1;
    p_sequencer.dmac.start_op(o);
    wr(R_DMA_BLK, blk);
    wr(R_DMA_CTRL, 32'h05);                                   // SEL = CMD, END_EN, EN
    wait (o.done);
    wait_cycles(40);                                          // final handshake over, end pulse ~400 cycles later
    chk(o.ack_log[o.ack_log.size() - 1] == DMA_OKAY, "final ack OKAY (DMAC without LAST_OKAY)");
    chk_rd(R_DMA_CTRL, 32'h05, '1, "END_EN=1: channel still enabled after the final OKAY");
    chk_rd(R_DMA_STAT, 0, 32'h0100_0100, "END_EN=1: no DONE / END before the end event");
    chk(n_done() == nd, "END_EN=1: no DMA_DONE before the end event");
    wait_op(o, 2ms, "END_EN=1 pulse");
    wait_ev(I_DMA_DONE, 1ms, 1, "END_EN=1 end pulse");
    chk(n_done() == nd + 1, "END_EN=1: exactly one DMA_DONE");
    chk_rd(R_DMA_STAT, 32'h0100_0100, 32'h0100_0100, "END_EN=1: DONE and END");
    chk_rd(R_DMA_CTRL, 32'h04, '1, "END_EN=1: EN cleared by the end event, END_EN kept");
    wait (cfg.dma_cfg.vif.req === 1'b0);                      // a request issued after the final OKAY: DENY (stopped DMAC)
    wait_cycles(4);
    chk_rd(R_DMA_STAT, 0, 32'h3, "END_EN=1: channel idle");
    wr(R_DMA_STAT, 32'h0101_0100);
    wait_cmpl(0, "END_EN=1 write");
    foreach (d[i]) chk(slv_mem(0, 8'hB0 + i) == d[i], "END_EN=1 data");

    // (c) end event while a request is pending (DMAC without DENY, END_EN = 1, 4-phase end): the channel completes,
    //     the request stays pending (the IP never withdraws it) until SW withdraws it with DMA_CTRL.CLR (4.9.7)
    cfg.dma_cfg.end_mode = END_HS;
    cfg.dma_cfg.end_lat_min = 2; cfg.dma_cfg.end_lat_max = 3;
    cfg.dma_cfg.no_deny = 1;
    nd = n_done();
    o = mk_op(SEL_TTX, 0, 2, 0);
    o.tx_data = '{32'h5A, 32'hA5};
    o.no_last_okay = 1;
    p_sequencer.dmac.start_op(o);
    wr(R_DMA_BLK, 0);
    wr(R_DMA_CTRL, 32'h25);                                   // SEL = TTX, END_EN, EN: SINGLE, SINGLE, SINGLE (pending)
    wait_op(o, 2ms, "end with a pending request");
    wait_ev(I_DMA_DONE, 1ms, 1, "end with a pending request");
    chk(n_done() == nd + 1, "pending request: one DMA_DONE");
    chk_rd(R_DMA_STAT, 32'h0100_0103, 32'h0101_0103, "DONE, END; the request issued before the end is still pending");
    chk_rd(R_DMA_CTRL, 32'h24, '1, "EN cleared by the end event");
    wait_cycles(200);
    chk(cfg.dma_cfg.vif.req === 1'b1, "request kept: the IP never withdraws it by itself");
    wr(R_DMA_CTRL, 32'h08);                                   // DMA_CTRL.CLR
    wait_cycles(1);
    chk(cfg.dma_cfg.vif.req === 1'b0, "request withdrawn by CLR");
    chk_rd(R_DMA_STAT, 32'h0100_0100, 32'h0101_0103, "CLR: REQ = 0, BUSY = 0, no DENY, DONE kept");
    chk_rd(R_FIFO_STAT, 2 << 16, 32'h3F << 16, "TTX holds the 2 bytes moved by the DMAC");
    wr(R_CTRL, 32'h0000_0403);                                // TTX_FLUSH
    wr(R_DMA_STAT, 32'h0101_0100);
    cfg.dma_cfg.no_deny = 0;

    // (d) END_EN = 1 but EN = 0: end events (pulse and 4-phase) are only recorded
    nd = n_done();
    p_sequencer.dmac.send_end(1);
    p_sequencer.dmac.send_end(0, 2);
    chk_rd(R_DMA_STAT, 32'h0100_0000, 32'h0100_0100, "EN = 0: END recorded, no DONE");
    chk(n_done() == nd, "EN = 0: no DMA_DONE");
    wr(R_DMA_STAT, 32'h0100_0000);

    // (e) END_EN locked while EN = 1; an end event on an enabled channel without any request completes it
    wr(R_DMA_CTRL, 32'h15);                                   // SEL = CRX (empty: no request), END_EN, EN
    wr(R_DMA_CTRL, 32'h11);                                   // END_EN = 0 ignored while EN = 1
    chk_rd(R_DMA_CTRL, 32'h15, '1, "END_EN locked while EN = 1");
    nd = n_done();
    p_sequencer.dmac.send_end(0);
    wait_ev(I_DMA_DONE, 100us, 1, "end event on an idle enabled channel");
    chk(n_done() == nd + 1, "one DMA_DONE");
    chk_rd(R_DMA_CTRL, 32'h14, '1, "EN cleared, END_EN kept");
    chk_rd(R_DMA_STAT, 32'h0100_0100, 32'h0100_0103, "DONE, END");
    wr(R_DMA_STAT, 32'h0101_0100);
    wr(R_DMA_CTRL, 32'h0);
    chk_rd(R_DMA_CTRL, 0, '1, "END_EN writable again");
    cfg.dma_cfg.end_lat_min = 0; cfg.dma_cfg.end_lat_max = 3;
  endtask
endclass : i2c_dma_end_vseq

// ---------------------------------------------------------------- VP-DMA-13: request withdrawn by SW, DMAC without DENY (4.9.7)
class i2c_dma_clr_vseq extends i2c_ip_dma_base_vseq;
  `uvm_object_utils(i2c_dma_clr_vseq)
  function new(string name = "i2c_dma_clr_vseq"); super.new(name); endfunction

  function int unsigned n_done();
    return p_sequencer.side.ev_cnt[I_DMA_DONE];
  endfunction

  task main();
    i2c_dma_op o;
    int unsigned nd, nl;
    int unsigned blk = (depth >= 8) ? 4 : 2;

    // (a) DMAC without DENY, channel stopped: the request stays pending; configuration locked; CLR withdraws it
    cfg.dma_cfg.no_deny = 1;
    p_sequencer.dmac.stop_op();
    wr(R_DMA_BLK, blk);
    wr(R_DMA_CTRL, 32'h21);                                   // SEL = TTX (empty) -> BLOCK request
    wait (cfg.dma_cfg.vif.req === 1'b1);
    wait_cycles(300);
    chk(cfg.dma_cfg.vif.req === 1'b1 && cfg.dma_cfg.vif.req_type == DMA_BLOCK, "BLOCK request pending (no DENY)");
    chk_rd(R_DMA_STAT, 32'h3, 32'h0101_0103, "REQ = 1, BUSY = 1, no DENY");
    wr(R_DMA_CTRL, 32'h16);                                   // EN = 0; SEL / PFC / END_EN locked (BUSY)
    chk_rd(R_DMA_CTRL, 32'h20, '1, "SEL / PFC / END_EN locked while the request is pending");
    wr(R_DMA_CTRL, 32'h08);                                   // CLR
    wait_cycles(1);
    chk(cfg.dma_cfg.vif.req === 1'b0, "request withdrawn the cycle after the CLR write");
    chk_rd(R_DMA_STAT, 0, 32'h0101_0103, "CLR: REQ = 0, BUSY = 0, no DONE / DENY");
    chk_rd(R_DMA_CTRL, 32'h20, '1, "CLR reads 0");
    wr(R_DMA_CTRL, 32'h16);
    chk_rd(R_DMA_CTRL, 32'h16, '1, "configuration writable after the withdrawal");
    wr(R_DMA_CTRL, 32'h0);

    // (b) CLR without a pending request: no effect
    wr(R_DMA_CTRL, 32'h08);
    wait_cycles(5);
    chk(cfg.dma_cfg.vif.req === 1'b0, "CLR in IDLE: no request");
    chk_rd(R_DMA_STAT, 0, '1, "CLR in IDLE: DMA_STAT unchanged");

    // (c) CLR with EN kept at 1: withdrawn, then issued again once the handshake is back in IDLE
    wr(R_DMA_CTRL, 32'h21);
    wait (cfg.dma_cfg.vif.req === 1'b1);
    wait_cycles(10);
    wr(R_DMA_CTRL, 32'h29);                                   // CLR, EN = 1
    wait_cycles(1);
    chk(cfg.dma_cfg.vif.req === 1'b0, "withdrawn with EN = 1");
    wait_cycles(6);
    chk(cfg.dma_cfg.vif.req === 1'b1, "request issued again (EN = 1)");
    wr(R_DMA_CTRL, 32'h08);                                   // CLR, EN = 0
    wait_cycles(4);
    chk_rd(R_DMA_STAT, 0, 32'h3, "withdrawn again, channel idle");
    cfg.dma_cfg.no_deny = 0;

    // (d) DMAC already serving when the request is withdrawn (4.9.7, SW condition violated): the elements moved by
    //     the DMAC reach the FIFO, the late ack (LAST_OKAY) is ignored - no DONE / DMA_DONE, channel back to IDLE
    cfg.dma_cfg.ack_after_withdraw = 1;
    cfg.dma_cfg.end_mode = END_NONE;
    cfg.dma_cfg.lat_min = 60; cfg.dma_cfg.lat_max = 60;
    nd = n_done();
    nl = p_sequencer.dmac.n_late_ack;
    o = mk_op(SEL_TTX, 0, blk, blk);
    for (int i = 0; i < blk; i++) o.tx_data.push_back(32'h30 + i);
    p_sequencer.dmac.start_op(o);
    wr(R_DMA_CTRL, 32'h21);                                   // BLOCK request
    wait (cfg.dma_cfg.vif.req === 1'b1);
    wait_cycles(5);
    wr(R_DMA_CTRL, 32'h08);                                   // withdrawn while the DMAC arbitrates
    fork
      begin                                                   // late ack seen in IDLE: the handshake waits in ACK
        @(posedge side.clk iff (cfg.dma_cfg.vif.ack === 1'b1 && cfg.dma_cfg.vif.req === 1'b0));
        repeat (2) @(posedge side.clk);                       // debug_dma_state_o is registered
        chk(side.d_dma_state == 2, "late ack in IDLE: handshake waits for ack = 0 in ACK (debug_dma_state_o = 2)");
      end
    join_none
    wait_op(o, 1ms, "late ack");
    wait_cycles(6);
    chk(p_sequencer.dmac.n_late_ack == nl + 1, "the DMAC acknowledged after the withdrawal");
    chk(n_done() == nd, "late LAST_OKAY not processed: no DMA_DONE");
    chk_rd(R_DMA_STAT, 0, 32'h0101_0103, "late ack ignored: no DONE / DENY, channel idle");
    chk_rd(R_FIFO_STAT, blk << 16, 32'h3F << 16, "elements moved by the DMAC are in TTX");
    wr(R_CTRL, 32'h0000_0403);                                // TTX_FLUSH

    // (e) ack sampled in the same cycle as the CLR write: the ack wins (handshake completes, ack_type processed)
    cfg.dma_cfg.ack_after_withdraw = 0;
    cfg.dma_cfg.lat_min = 200; cfg.dma_cfg.lat_max = 200;     // the DMAC model stays in its arbitration delay
    p_sequencer.dmac.stop_op();
    wr(R_DMA_CTRL, 32'h21);
    wait (cfg.dma_cfg.vif.req === 1'b1);
    fork
      wr(R_DMA_CTRL, 32'h08);
      begin                                                   // ack = 1 (DENY) during the access phase of that write
        do @(posedge side.clk);
        while (!(cfg.apb_cfg.vif.psel === 1'b1 && cfg.apb_cfg.vif.penable === 1'b0 && cfg.apb_cfg.vif.pwrite === 1'b1 &&
                 cfg.apb_cfg.vif.paddr[11:0] == R_DMA_CTRL));
        cfg.dma_cfg.vif.ack <= 1'b1;
        cfg.dma_cfg.vif.ack_type <= DMA_DENY;
        do @(posedge side.clk); while (cfg.dma_cfg.vif.req !== 1'b0);
        cfg.dma_cfg.vif.ack <= 1'b0;
        cfg.dma_cfg.vif.ack_type <= 2'b00;
      end
    join
    wait_cycles(4);
    chk_rd(R_DMA_STAT, 32'h0001_0000, 32'h0001_0103, "ack and CLR in the same cycle: the ack wins (DENY processed)");
    wait_cycles(250);                                         // the DMAC model leaves its delay (request gone: no ack)
    wr(R_DMA_STAT, 32'h0101_0100);
    wr(R_DMA_CTRL, 32'h0);
    cfg.dma_cfg.lat_min = 2; cfg.dma_cfg.lat_max = 6;
    cfg.dma_cfg.end_mode = END_HS;
  endtask
endclass : i2c_dma_clr_vseq
