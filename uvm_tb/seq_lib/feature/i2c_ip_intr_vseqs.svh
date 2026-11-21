// -----------------------------------------------------------------------------
// Layer 2 - interrupt / FIFO feature sequences (spec 3.7, 4.7, 4.8, 5.2.3 - 5.2.10)
// -----------------------------------------------------------------------------

class i2c_ip_intr_base_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_intr_base_vseq)
  function new(string name = "i2c_ip_intr_base_vseq"); super.new(name); endfunction

  task chk_lines(bit o, bit c, bit t, bit e, string what);
    wait_cycles(3);
    chk(side.irq === o && side.irq_ctrl === c && side.irq_tgt === t && side.irq_err === e,
        $sformatf("%s: irq_o/ctrl/tgt/err = %b%b%b%b, expected %b%b%b%b", what, side.irq, side.irq_ctrl, side.irq_tgt, side.irq_err, o, c, t, e));
  endtask
endclass : i2c_ip_intr_base_vseq

// ---------------------------------------------------------------- VP-INT-01: event bits, W1C, INTR_EN routing (4.8)
class i2c_intr_event_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_intr_event_vseq)
  function new(string name = "i2c_intr_event_vseq"); super.new(name); endfunction

  bit shuffle = 0;                     // random order of the event bits

  task main();
    int bits[$];
    for (int b = 0; b < 22; b++) if (EVENT_MASK[b]) bits.push_back(b);
    if (shuffle) bits.shuffle();
    foreach (bits[k]) begin
      int b = bits[k];
      bit [3:0] lane_off = ~(4'b0001 << (b / 8));
      wr(R_INTR_TEST, 32'h1 << b);
      chk_ev(b, 1, $sformatf("INTR_TEST bit %0d:", b));
      chk_lines(0, 0, 0, 0, $sformatf("bit %0d masked", b));
      wr(R_INTR_EN, 32'h1 << b);
      chk_lines(1, b < 8, b >= 8 && b < 16, b >= 16 && b < 21, $sformatf("bit %0d enabled", b));
      wr(R_INTR_STAT, ~(32'h1 << b) & ~EVENT_MASK);           // W1C of other (level / reserved) bits: no effect
      chk_ev(b, 1, $sformatf("bit %0d after W1C of non-event bits:", b));
      wr(R_INTR_STAT, 32'h1 << b, lane_off);                   // its byte lane not strobed: no effect (3.6)
      chk_ev(b, 1, $sformatf("bit %0d after W1C with its lane not strobed:", b));
      wr(R_INTR_STAT, 32'h1 << b);
      chk_ev(b, 0, $sformatf("bit %0d after W1C:", b));
      chk_lines(0, 0, 0, 0, $sformatf("bit %0d cleared", b));
      wr(R_INTR_EN, 0);
    end
  endtask
endclass : i2c_intr_event_vseq

// ---------------------------------------------------------------- VP-INT-02: several events at once, masking (4.8)
class i2c_intr_multi_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_intr_multi_vseq)
  function new(string name = "i2c_intr_multi_vseq"); super.new(name); endfunction

  int unsigned n_iter = 20;

  task main();
    repeat (n_iter) begin
      bit [31:0] ev = $urandom & EVENT_MASK;
      bit [31:0] en = $urandom & 32'h003F_FFFF & ~32'h0000_0303;  // level bits (always 1 / 0 here) excluded
      bit [31:0] act = ev & en;
      wr(R_INTR_TEST, ev);
      chk_rd(R_INTR_STAT, ev | 32'h201, '1, "INTR_STAT after a random INTR_TEST");
      wr(R_INTR_EN, en);
      chk_lines(|act, |act[7:0], |act[15:8], |act[20:16], $sformatf("INTR_EN 0x%06h events 0x%06h", en, ev));
      wr(R_INTR_EN, 0);
      chk_lines(0, 0, 0, 0, "INTR_EN = 0 masks every line");
      wr(R_INTR_STAT, ev);
      chk_rd(R_INTR_STAT, 32'h201, '1, "all events cleared by one W1C");
    end
  endtask
endclass : i2c_intr_multi_vseq

// ---------------------------------------------------------------- VP-INT-03: level bits (4.8)
class i2c_intr_level_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_intr_level_vseq)
  function new(string name = "i2c_intr_level_vseq"); super.new(name); endfunction

  task main();
    wr(R_INTR_TEST, 32'h0000_0303);
    chk_rd(R_INTR_STAT, 32'h0000_0201, '1, "level bits unaffected by INTR_TEST");
    wr(R_INTR_STAT, 32'h0000_0303);
    chk_rd(R_INTR_STAT, 32'h0000_0201, '1, "level bits unaffected by W1C");
    wr(R_INTR_EN, 32'h0000_0001);
    chk_lines(1, 1, 0, 0, "level CMD_THR enabled");
    wr(R_INTR_EN, 32'h0000_0200);
    chk_lines(1, 0, 1, 0, "level TTX_THR enabled");
    wr(R_INTR_EN, 0);
  endtask
endclass : i2c_intr_level_vseq

// ---------------------------------------------------------------- VP-FIFO-01: CMD FIFO (4.7, 3.7, 5.2.4)
class i2c_fifo_cmd_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_fifo_cmd_vseq)
  function new(string name = "i2c_fifo_cmd_vseq"); super.new(name); endfunction

  task main();
    bit e;
    wr(R_CTRL, 32'h1);                                       // EN only: commands are not executed
    for (int i = 0; i < depth; i++) begin
      wr(R_FIFO_CTRL, i);                                     // CMD_THR = i : bit = (lvl <= i)
      chk_ev(I_CMD_THR, 1, $sformatf("CMD_THR with lvl %0d <= thr %0d:", i, i));
      wr(R_CMD, c_start(7'h50, 0));
      chk_ev(I_CMD_THR, 0, $sformatf("CMD_THR with lvl %0d > thr %0d:", i + 1, i));
    end
    chk_rd(R_FIFO_STAT, depth, 32'h3F, "CMD_LVL = FIFO_DEPTH");
    chk_rd(R_STATUS, 32'h0000_A900, 32'h0000_FF00, "STATUS: CMD_FULL, others empty");
    wr(R_FIFO_CTRL, 63);                                      // clamped to FIFO_DEPTH - 1 (5.2.4)
    chk_ev(I_CMD_THR, 0, "CMD_THR clamped: lvl = depth > depth-1:");
    wr_e(R_CMD, 32'h1A0, e);
    chk(e, "write to a full CMD FIFO answers PSLVERR");
    chk_ev(I_FIFO_ERR, 1, "FIFO_ERR after CMD overflow:");
    chk_rd(R_FIFO_STAT, depth, 32'h3F, "CMD_LVL unchanged after overflow");
    w1c(32'h1 << I_FIFO_ERR);
    wr(R_CTRL, 32'h0);                                        // EN 1 -> 0: FIFO content kept (4.7)
    chk_rd(R_FIFO_STAT, depth, 32'h3F, "CMD content kept with EN = 0");
    wr(R_CTRL, 32'h0000_0100);                                // CMD_FLUSH
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD_FLUSH");
    chk_ev(I_CMD_THR, 1, "CMD_THR after flush:");
    wr(R_CMD, 32'h1A0, 4'b0000);                              // PSTRB = 0: no push (3.6)
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD write with PSTRB = 0 does not push");
    wr(R_CMD, 32'h1A0, 4'b0010);                              // one lane: push
    chk_rd(R_FIFO_STAT, 1, 32'h3F, "CMD write with one lane pushes");
    wr(R_CTRL, 32'h0000_0100);
    wr(R_FIFO_CTRL, 0);
  endtask
endclass : i2c_fifo_cmd_vseq

// ---------------------------------------------------------------- VP-FIFO-02: TTX FIFO (4.7, 3.7)
class i2c_fifo_ttx_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_fifo_ttx_vseq)
  function new(string name = "i2c_fifo_ttx_vseq"); super.new(name); endfunction

  task main();
    bit e;
    for (int i = 0; i < depth; i++) begin
      wr(R_FIFO_CTRL, i << 16);
      chk_ev(I_TTX_THR, 1, $sformatf("TTX_THR with lvl %0d <= thr:", i));
      wr(R_TTXDATA, i);
      chk_ev(I_TTX_THR, 0, $sformatf("TTX_THR with lvl %0d > thr:", i + 1));
    end
    chk_rd(R_FIFO_STAT, depth << 16, 32'h3F << 16, "TTX_LVL = FIFO_DEPTH");
    chk_rd(R_STATUS, 32'h0000_9A00, 32'h0000_FF00, "STATUS: TTX_FULL");
    wr_e(R_TTXDATA, 32'h55, e);
    chk(e, "write to a full TTX FIFO answers PSLVERR");
    chk_ev(I_FIFO_ERR, 1, "FIFO_ERR after TTX overflow:");
    w1c(32'h1 << I_FIFO_ERR);
    wr(R_CTRL, 32'h0000_0400);                                // TTX_FLUSH
    chk_rd(R_FIFO_STAT, 0, '1, "TTX_FLUSH");
    wr(R_FIFO_CTRL, 0);
  endtask
endclass : i2c_fifo_ttx_vseq

// ---------------------------------------------------------------- VP-FIFO-03: reads of empty RX FIFOs (3.7)
class i2c_fifo_rx_empty_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_fifo_rx_empty_vseq)
  function new(string name = "i2c_fifo_rx_empty_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v;
    bit e;
    rd_e(R_CRXDATA, v, e); chk(e && v == 0, "read of empty CRX: PSLVERR, 0");
    chk_ev(I_FIFO_ERR, 1, "FIFO_ERR after CRX underflow:"); w1c(32'h1 << I_FIFO_ERR);
    rd_e(R_TRXDATA, v, e); chk(e && v == 0, "read of empty TRX: PSLVERR, 0");
    chk_ev(I_FIFO_ERR, 1, "FIFO_ERR after TRX underflow:"); w1c(32'h1 << I_FIFO_ERR);
    // FIFO_ERR on irq_err_o
    wr(R_INTR_EN, 32'h0010_0000);
    rd_e(R_TRXDATA, v, e);
    chk_lines(1, 0, 0, 1, "FIFO_ERR enabled");
    w1c(EVENT_MASK);
    chk_lines(0, 0, 0, 0, "FIFO_ERR cleared");
    // DMA_DONE only reaches irq_o (4.8)
    wr(R_INTR_EN, 32'h0020_0000);
    wr(R_INTR_TEST, 32'h0020_0000);
    chk_lines(1, 0, 0, 0, "DMA_DONE only on irq_o");
    w1c(EVENT_MASK);
    wr(R_INTR_EN, 0);
  endtask
endclass : i2c_fifo_rx_empty_vseq

// ---------------------------------------------------------------- VP-FIFO-04: CRX threshold with real traffic (4.7)
//   needs the controller enabled; reads `n` bytes into CRX with CRX_THR = thr, then drains them one by one
class i2c_fifo_crx_thr_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_fifo_crx_thr_vseq)
  function new(string name = "i2c_fifo_crx_thr_vseq"); super.new(name); endfunction

  rand int unsigned thr = 2, n = 3;
  constraint c_def { soft thr == 2; soft n == 3; n inside {[1:32]}; thr inside {[0:31]}; }

  task main();
    bit [12:0] c[$];
    bit [31:0] v;
    bit e;
    if (n > depth) n = depth;
    if (thr >= depth) thr = depth - 1;
    wr(R_FIFO_CTRL, thr << 8);
    c = '{c_start(7'h50, 0), c_wr(8'h10), c_start(7'h50, 1)};
    for (int i = 0; i < n; i++) c.push_back(c_rd(0, i == n - 1));
    push_cmds(c);
    wait_cmpl(0, $sformatf("read %0d bytes", n));
    wr_e(R_CRXDATA, 32'hFFFF_FFFF, e);                        // RO data register: write ignored, no pop (3.7)
    chk(!e, "write to CRXDATA: no PSLVERR");
    chk_rd(R_FIFO_STAT, n << 8, 32'h3F << 8, "write to CRXDATA does not pop");
    wr(R_INTR_EN, 32'h2);
    for (int lvl = n; lvl >= 0; lvl--) begin
      chk_ev(I_CRX_THR, lvl > thr, $sformatf("CRX_THR with level %0d, threshold %0d:", lvl, thr));
      chk_lines(lvl > thr, lvl > thr, 0, 0, $sformatf("irq_ctrl_o by CRX_THR (level %0d)", lvl));
      if (lvl > 0) rd(R_CRXDATA, v);
    end
    wr(R_INTR_EN, 0);
    wr(R_FIFO_CTRL, 0);
  endtask
endclass : i2c_fifo_crx_thr_vseq

// ---------------------------------------------------------------- VP-FIFO-05: TRX threshold with real traffic (4.7)
class i2c_fifo_trx_thr_vseq extends i2c_ip_intr_base_vseq;
  `uvm_object_utils(i2c_fifo_trx_thr_vseq)
  function new(string name = "i2c_fifo_trx_thr_vseq"); super.new(name); endfunction

  rand int unsigned thr = 2, n = 2;    // n data bytes -> n + 2 TRX entries
  constraint c_def { soft thr == 2; soft n == 2; n inside {[1:14]}; thr inside {[0:31]}; }

  task main();
    bit [31:0] v;
    bit [7:0]  d[$];
    bit ok, e;
    int unsigned ne;
    if (n + 2 > depth) n = depth - 2;                         // the whole transaction fits in TRX
    if (thr >= depth) thr = depth - 1;
    ne = n + 2;
    wr(R_FIFO_CTRL, thr << 24);
    rnd_bytes(n, d);
    ext_write(own0, d, ok);
    chk(ok, "VIP master write to the DUT target");
    wr_e(R_TRXDATA, 32'hFFFF_FFFF, e);
    chk(!e, "write to TRXDATA: no PSLVERR");
    chk_rd(R_FIFO_STAT, ne << 24, 32'h3F << 24, "write to TRXDATA does not pop");
    wr(R_INTR_EN, 32'h100);
    for (int lvl = ne; lvl >= 0; lvl--) begin
      chk_ev(I_TRX_THR, lvl > thr, $sformatf("TRX_THR with level %0d, threshold %0d:", lvl, thr));
      chk_lines(lvl > thr, 0, lvl > thr, 0, $sformatf("irq_tgt_o by TRX_THR (level %0d)", lvl));
      if (lvl > 0) rd(R_TRXDATA, v);
    end
    wr(R_INTR_EN, 0);
    wr(R_FIFO_CTRL, 0);
    w1c(EVENT_MASK);
  endtask
endclass : i2c_fifo_trx_thr_vseq
