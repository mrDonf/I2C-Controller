// -----------------------------------------------------------------------------
// Layer 2 - register / APB feature sequences (spec 3.1, 3.6, 3.7, 5.1, 5.2, 6.1.1)
//   Each sequence checks one item of the verification plan; the tests compose them with the standard UVM register
//   sequences (uvm_reg_hw_reset_seq, uvm_reg_bit_bash_seq) and the layer-1 sequences.
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- VP-REG-01: reset values of the spec table (5.1)
class i2c_reg_reset_val_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_reset_val_vseq)
  function new(string name = "i2c_reg_reset_val_vseq"); super.new(name); endfunction

  task main();
    foreach (ral.all_regs[i]) begin
      i2c_ip_reg r = ral.all_regs[i];
      bit [11:0] a = r.get_offset();
      bit [31:0] v, exp;
      bit e;
      string acc = r.get_rights();
      bit pop = (a == R_CRXDATA) || (a == R_TRXDATA);            // reading an empty RX FIFO: PSLVERR, 0 (3.7)
      exp = (acc == "WO") ? 32'h0 : 32'(r.get_reset());         // a WO register reads 0 (3.7)
      rd_e(a, v, e);
      chk(e == pop, $sformatf("%s: PSLVERR = %0d on read, expected %0d", r.get_name(), e, pop));
      chk(v === exp, $sformatf("%s reset value 0x%08h, spec 0x%08h", r.get_name(), v, exp));
    end
    chk_ev(I_FIFO_ERR, 1, "read of an empty CRX/TRX sets FIFO_ERR:");
    w1c(32'h1 << I_FIFO_ERR);
    // explicit values of the spec table (independent of the register model)
    chk_rd(R_STATUS,    32'h0000_AA00, '1, "STATUS");
    chk_rd(R_INTR_STAT, 32'h0000_0201, '1, "INTR_STAT");
    chk_rd(R_HSCFG,     32'h0000_0001, '1, "HSCFG");
    chk_rd(R_TIM_SCL,   32'h01EA_01F4, '1, "TIM_SCL");
    chk_rd(R_TIM_STA,   32'h01D6_0190, '1, "TIM_STA");
    chk_rd(R_TIM_STO,   32'h01D6_0190, '1, "TIM_STO");
    chk_rd(R_TIM_DAT,   32'h007D_001E, '1, "TIM_DAT");
    chk_rd(R_HS_TIM_SCL,32'h0005_000F, '1, "HS_TIM_SCL");
    chk_rd(R_HS_TIM_STA,32'h0010_0010, '1, "HS_TIM_STA");
    chk_rd(R_HS_TIM_STO,32'h0000_0010, '1, "HS_TIM_STO");
    chk_rd(R_HS_TIM_DAT,32'h0009_0001, '1, "HS_TIM_DAT");
    chk_rd(R_FILTER,    32'h0000_0105, '1, "FILTER");
    chk_rd(R_TIMEOUT,   32'h0035_67E0, '1, "TIMEOUT");
    chk_rd(R_BUSIDLE,   32'h0000_03E8, '1, "BUSIDLE");
    chk_rd(R_DEVID,     32'(cfg.p.devid_rst), '1, "DEVID");
    chk_rd(R_BUSMON,    32'h0000_0603, '1, "BUSMON");
    chk_rd(R_PARAM,     32'h0000_1F00 | depth, '1, "PARAM");
    chk_rd(R_VERSION,   cfg.p.version, '1, "VERSION");
  endtask
endclass : i2c_reg_reset_val_vseq

// ---------------------------------------------------------------- VP-REG-01: outputs after reset (6.1.1)
class i2c_reset_outputs_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reset_outputs_vseq)
  function new(string name = "i2c_reset_outputs_vseq"); super.new(name); endfunction

  task main();
    chk(side.scl_oe === 1'b0 && side.sda_oe === 1'b0, "scl_oe_o / sda_oe_o = 0 after reset");
    chk(side.irq === 1'b0 && side.irq_ctrl === 1'b0 && side.irq_tgt === 1'b0 && side.irq_err === 1'b0, "irq_* = 0 after reset");
    chk(side.cs_en === 1'b0 && side.hs_mode === 1'b0, "scl_cs_en_o / hs_mode_o = 0 after reset");
    chk(cfg.dma_cfg.vif.req === 1'b0, "dma_req_o = 0 after reset");
    chk(cfg.dma_cfg.vif.end_ack === 1'b0, "dma_end_ack_o = 0 after reset");
  endtask
endclass : i2c_reset_outputs_vseq

// ---------------------------------------------------------------- VP-REG-02: non-reset state before a PRESETn pulse
class i2c_reg_dirty_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_dirty_vseq)
  function new(string name = "i2c_reg_dirty_vseq"); super.new(name); endfunction

  task main();
    wr(R_TADDR, 32'h0000_8000 | $urandom_range(16'h7FFF, 1));
    wr(R_INTR_EN, 32'h003F_FFFF);
    wr(R_INTR_TEST, EVENT_MASK);
    wr(R_TCFG, $urandom_range(15, 1));
    wr(R_HSCFG, $urandom_range(7, 2));
    wr(R_TIM_SCL, $urandom);
    wr(R_FILTER, 32'h0000_0307);
    wr(R_DEVID, $urandom);
    wr(R_TIMEOUT, 32'h8000_1234);
    wr(R_DMA_BLK, 3);
    wr(R_CTRL, 32'h0000_0001);
    for (int i = 0; i < 3; i++) wr(R_CMD, c_wr(i));            // CMD FIFO not empty (controller disabled)
    wr(R_TTXDATA, 8'h5A);
  endtask
endclass : i2c_reg_dirty_vseq

// ---------------------------------------------------------------- VP-REG-03: RW registers, reserved bits (5.1, 3.7)
class i2c_reg_rw_pattern_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_rw_pattern_vseq)
  function new(string name = "i2c_reg_rw_pattern_vseq"); super.new(name); endfunction

  int unsigned n_rand = 2;

  // value of a pattern restricted to the defined bits; FIFO thresholds / DMA_BLK stay below FIFO_DEPTH (the spec
  // leaves the read-back of a clamped value open - the clamping itself is checked functionally)
  function bit [31:0] legal(i2c_ip_reg r, bit [31:0] pat);
    bit [31:0] v = pat & 32'(r.defined_mask());
    if (r == ral.FIFO_CTRL) for (int k = 0; k < 4; k++) v[8*k +: 6] = v[8*k +: 6] % depth;
    if (r == ral.DMA_BLK) v[5:0] = v[5:0] % (depth + 1);
    return v;
  endfunction

  task main();
    i2c_ip_reg rw_regs[$];
    bit [31:0] pats[$];
    rw_regs = '{ral.INTR_EN, ral.FIFO_CTRL, ral.TADDR, ral.TCFG, ral.HSCFG, ral.TIM_SCL, ral.TIM_STA,
                ral.TIM_STO, ral.TIM_DAT, ral.HS_TIM_SCL, ral.HS_TIM_STA, ral.HS_TIM_STO, ral.HS_TIM_DAT,
                ral.FILTER, ral.TIMEOUT, ral.BUSIDLE, ral.DEVID, ral.DMA_BLK};
    pats = '{32'hFFFF_FFFF, 32'h0000_0000, 32'hAAAA_AAAA, 32'h5555_5555};
    repeat (n_rand) pats.push_back($urandom);
    wr(R_CTRL, 32'h0);
    foreach (rw_regs[i]) begin
      bit [11:0] a = rw_regs[i].get_offset();
      foreach (pats[k]) begin
        bit [31:0] x = legal(rw_regs[i], pats[k]);
        bit [31:0] w = (rw_regs[i] == ral.FIFO_CTRL || rw_regs[i] == ral.DMA_BLK) ? (x | (pats[k] & ~32'(rw_regs[i].defined_mask()))) : pats[k];
        wr(a, w);
        chk_rd(a, x, '1, {rw_regs[i].get_name(), " pattern"});
      end
      wr(a, 32'(rw_regs[i].get_reset()));
    end
  endtask
endclass : i2c_reg_rw_pattern_vseq

// ---------------------------------------------------------------- VP-REG-04: PSTRB byte lanes (3.6)
class i2c_reg_strobe_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_strobe_vseq)
  function new(string name = "i2c_reg_strobe_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v;
    // every lane combination on a 4-lane RW register
    wr(R_CTRL, 32'h0);
    for (int s = 0; s < 16; s++) begin
      bit [31:0] exp, pat = $urandom;
      wr(R_DEVID, 32'h0);
      wr(R_TIM_SCL, 32'h0);
      wr(R_TIM_SCL, pat, 4'(s));
      exp = 0;
      for (int k = 0; k < 4; k++) if (s[k]) exp[8*k +: 8] = pat[8*k +: 8];
      chk_rd(R_TIM_SCL, exp, '1, $sformatf("TIM_SCL after PSTRB = 4'b%04b", s));
    end
    // lane by lane accumulation, PSTRB = 0 writes nothing
    wr(R_TIM_SCL, 32'h0000_0000);
    for (int lane = 0; lane < 4; lane++) begin
      bit [31:0] exp = 32'h0;
      for (int k = 0; k <= lane; k++) exp[8*k +: 8] = 8'h11 * (k + 1);
      wr(R_TIM_SCL, 32'h4433_2211, 4'b0001 << lane);
      chk_rd(R_TIM_SCL, exp, '1, $sformatf("TIM_SCL after strobe lane %0d", lane));
    end
    wr(R_TIM_SCL, 32'h0000_0000, 4'b0000);
    chk_rd(R_TIM_SCL, 32'h4433_2211, '1, "TIM_SCL after PSTRB = 0");
    wr(R_TIM_SCL, 32'h01EA_01F4);
    wr(R_DEVID, cfg.p.devid_rst);
    // W1C through a non-strobed lane has no effect (INTR_STAT bit 12 is in lane 1)
    wr(R_INTR_TEST, 32'h1 << I_TSTOP);
    wr(R_INTR_STAT, 32'h1 << I_TSTOP, 4'b1101);
    chk_ev(I_TSTOP, 1, "W1C with the lane of the bit not strobed:");
    wr(R_INTR_STAT, 32'h1 << I_TSTOP, 4'b0010);
    chk_ev(I_TSTOP, 0, "W1C through the lane of the bit only:");
    // CMD: any strobed lane pushes, PSTRB = 0 does not (3.6)
    wr(R_CTRL, 32'h1);
    for (int s = 0; s < 16; s++) begin
      int unsigned lvl;
      wr(R_CMD, c_wr(8'(s)), 4'(s));
      rd_field(R_FIFO_STAT, 0, 6, lvl);
      chk(lvl == (s != 0), $sformatf("CMD write with PSTRB = 4'b%04b: level %0d", s, lvl));
      wr(R_CTRL, 32'h0000_0101);                             // CMD_FLUSH
    end
    wr(R_CTRL, 32'h0);
    rd(R_FIFO_STAT, v);
    chk(v == 0, "FIFOs empty after the strobe checks");
  endtask
endclass : i2c_reg_strobe_vseq

// ---------------------------------------------------------------- VP-REG-05: "RW (lock)" while CTRL.EN = 1 (3.7, 5.1)
class i2c_reg_lock_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_lock_vseq)
  function new(string name = "i2c_reg_lock_vseq"); super.new(name); endfunction

  task main();
    bit e;
    bit [31:0] c;
    rd(R_CTRL, c);
    wr(R_CTRL, c | 32'h1);
    foreach (ral.timing_regs[i]) begin
      bit [31:0] prev_v;
      bit [11:0] a = ral.timing_regs[i].get_offset();
      rd(a, prev_v);
      wr_e(a, ~prev_v, e);
      chk(!e, {ral.timing_regs[i].get_name(), " locked write: no PSLVERR"});
      chk_rd(a, prev_v, '1, {ral.timing_regs[i].get_name(), " unchanged while EN = 1"});
      wr_e(a, ~prev_v, e, 4'b0001);
      chk_rd(a, prev_v, '1, {ral.timing_regs[i].get_name(), " unchanged by a partial write while EN = 1"});
    end
    begin
      bit [31:0] t0, d0;
      rd(R_TIMEOUT, t0); rd(R_DEVID, d0);
      wr(R_TIMEOUT, 32'h8000_1234);  chk_rd(R_TIMEOUT, 32'h8000_1234, '1, "TIMEOUT writable with EN = 1");
      wr(R_DEVID, 32'h00AB_CDEF);    chk_rd(R_DEVID, 32'h00AB_CDEF, '1, "DEVID writable with EN = 1");
      wr(R_TIMEOUT, t0); wr(R_DEVID, d0);
    end
    wr(R_CTRL, 32'h0);
    begin
      bit [31:0] s0;
      rd(R_TIM_STA, s0);
      wr(R_TIM_STA, 32'h0033_0022);  chk_rd(R_TIM_STA, 32'h0033_0022, '1, "TIM_STA writable again with EN = 0");
      wr(R_TIM_STA, s0);
    end
    wr(R_CTRL, c & 32'h1F);
  endtask
endclass : i2c_reg_lock_vseq

// ---------------------------------------------------------------- VP-REG-06: RO / WO registers (3.7)
class i2c_reg_ro_wo_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_ro_wo_vseq)
  function new(string name = "i2c_reg_ro_wo_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v;
    bit e;
    foreach (ral.all_regs[i]) begin
      i2c_ip_reg r = ral.all_regs[i];
      if (r.get_rights() == "RO" && r != ral.CRXDATA && r != ral.TRXDATA) begin
        bit [31:0] prev_v;
        rd(r.get_offset(), prev_v);
        wr_e(r.get_offset(), 32'hFFFF_FFFF, e);
        chk(!e, {r.get_name(), " (RO) write: no PSLVERR"});
        chk_rd(r.get_offset(), prev_v, '1, {r.get_name(), " (RO) unchanged after write"});
      end
    end
    rd_e(R_INTR_TEST, v, e); chk(!e && v == 0, "INTR_TEST reads 0 without PSLVERR");
    rd_e(R_CMD, v, e);       chk(!e && v == 0, "CMD reads 0 without PSLVERR");
    rd_e(R_TTXDATA, v, e);   chk(!e && v == 0, "TTXDATA reads 0 without PSLVERR");
    chk_rd(R_FIFO_STAT, 0, '1, "reads of WO FIFO registers pop nothing");
  endtask
endclass : i2c_reg_ro_wo_vseq

// ---------------------------------------------------------------- VP-REG-07: unmapped offsets (3.7)
class i2c_reg_unmapped_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_unmapped_vseq)
  function new(string name = "i2c_reg_unmapped_vseq"); super.new(name); endfunction

  int unsigned n_rand = 8;

  task main();
    bit [11:0] unm[$];
    bit [31:0] v, c0;
    bit e;
    unm = '{12'h080, 12'h0A4, 12'h0F4, 12'h100, 12'h400, 12'hFFC};
    repeat (n_rand) begin                                      // random unmapped offsets (0x080..0x0F4, 0x100..0xFFC)
      bit [11:0] a = $urandom_range(1) ? 12'($urandom_range('h080, 'h0F4)) : 12'($urandom_range('h100, 'hFFC));
      unm.push_back(a & 12'hFFC);
    end
    rd(R_CTRL, c0);
    foreach (unm[i]) begin
      wr_e(unm[i], 32'hFFFF_FFFF, e); chk(e, $sformatf("write to unmapped 0x%03h answers PSLVERR", unm[i]));
      rd_e(unm[i], v, e);             chk(e && v == 0, $sformatf("read of unmapped 0x%03h: PSLVERR and 0 (got %0d, 0x%08h)", unm[i], e, v));
    end
    chk_rd(R_CTRL, c0, '1, "CTRL untouched by unmapped writes");
    chk_ev(I_FIFO_ERR, 0, "unmapped accesses do not set FIFO_ERR:");
  endtask
endclass : i2c_reg_unmapped_vseq

// ---------------------------------------------------------------- VP-REG-08: PADDR[1:0] / PPROT ignored (3.1)
class i2c_reg_paddr_pprot_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_paddr_pprot_vseq)
  function new(string name = "i2c_reg_paddr_pprot_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v, t0;
    bit e;
    rd(R_TADDR, t0);
    for (int lo = 0; lo < 4; lo++) begin
      bit [15:0] val = 16'($urandom);
      wr(R_TADDR | lo, val);
      chk_rd(R_TADDR, val, '1, $sformatf("write at 0x%03h lands in TADDR", R_TADDR | lo));
      rd(R_TADDR | (3 - lo), v);
      chk(v == val, $sformatf("read at 0x%03h returns TADDR (0x%08h)", R_TADDR | (3 - lo), v));
    end
    for (int p = 0; p < 8; p++) begin
      wr_e(R_TADDR, 32'h0000_0010 + p, e, 4'hF, 3'(p));
      chk(!e, $sformatf("write with PPROT = %0d accepted", p));
      rd_e(R_TADDR, v, e, 3'(p));
      chk(!e && v == 32'h10 + p, $sformatf("read with PPROT = %0d", p));
    end
    wr(R_TADDR, t0);
  endtask
endclass : i2c_reg_paddr_pprot_vseq

// ---------------------------------------------------------------- VP-REG-09: W1C / W1SC / DMA_CTRL lane rule (5.2)
class i2c_reg_w1c_w1s_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_reg_w1c_w1s_vseq)
  function new(string name = "i2c_reg_w1c_w1s_vseq"); super.new(name); endfunction

  task main();
    wr(R_CSTAT, 32'h7F);        chk_rd(R_CSTAT, 0, '1, "CSTAT W1C with nothing pending");
    wr(R_DMA_STAT, 32'h1_0100); chk_rd(R_DMA_STAT, 0, '1, "DMA_STAT W1C with nothing pending");
    wr(R_CTRL, 32'h0000_0F18);                               // flushes + HS_EN + STARTBYTE_EN
    chk_rd(R_CTRL, 32'h0000_0018, '1, "CTRL: flush bits self-clear, HS_EN / STARTBYTE_EN stored");
    wr(R_CTRL, 32'h0);
    // DMA_CTRL: SEL / PFC written with EN = 0, through lane 0 only (5.2.22)
    wr(R_DMA_CTRL, 32'h0000_0032);
    chk_rd(R_DMA_CTRL, 32'h0000_0032, '1, "DMA_CTRL SEL = 3, PFC = 1");
    wr(R_DMA_CTRL, 32'h0000_0000, 4'b1110);
    chk_rd(R_DMA_CTRL, 32'h0000_0032, '1, "DMA_CTRL written only through lane 0");
    wr(R_DMA_CTRL, 32'h0000_002C);                              // END_EN = 1, CLR = 1 (no request: no effect)
    chk_rd(R_DMA_CTRL, 32'h0000_0024, '1, "DMA_CTRL END_EN = 1, CLR reads 0");
    wr(R_DMA_CTRL, 32'h0);
    // register model front-door + mirror check
    begin
      uvm_status_e st;
      reg_wr(ral.TCFG, 4'hA);
      ral.TCFG.mirror(st, UVM_CHECK, .parent(this));
      chk(st == UVM_IS_OK, "RAL mirror of TCFG");
      reg_wr(ral.TCFG, 0);
    end
  endtask
endclass : i2c_reg_w1c_w1s_vseq
