// -----------------------------------------------------------------------------
// i2c_ip_coverage - functional coverage of the apb_i2c features (spec 1.3 F1..F20)
//   cg_cmd     controller commands written to CMD (4.4.1): START/STOP/READ/NACK/NAKOK combinations
//   cg_ctrl    bus messages driven by the DUT controller: direction x length x address ACK x Sr x speed, prefixes
//   cg_tgt     external messages seen by the DUT target: address class (4.5.1) x direction x DUT ACK
//   cg_trx     TRX entries read: TAG (4.5.2)
//   cg_crx     CRX entries read: LAST
//   cg_intr    every INTR_STAT bit observed set (4.8)
//   cg_dma     DMA handshakes: FIFO (SEL) x req_type x ack_type (4.9)
//   cg_dma_ev  DMA channel events (4.9.6, 4.9.7): end handshake (4-phase / pulse) x DMA_CTRL.END_EN x request pending,
//              request withdrawn by DMA_CTRL.CLR, late ack
//   cg_apb     register accesses: offset class x direction x PSLVERR x strobes (3.6, 3.7)
// -----------------------------------------------------------------------------
`uvm_analysis_imp_decl(_cov_apb)
`uvm_analysis_imp_decl(_cov_msg)
`uvm_analysis_imp_decl(_cov_side)
`uvm_analysis_imp_decl(_cov_dma)

class i2c_ip_coverage extends uvm_component;
  `uvm_component_utils(i2c_ip_coverage)

  uvm_analysis_imp_cov_apb  #(apb_item, i2c_ip_coverage)         apb_imp;
  uvm_analysis_imp_cov_msg  #(i2c_ip_msg_info, i2c_ip_coverage)  msg_imp;
  uvm_analysis_imp_cov_side #(i2c_ip_side_item, i2c_ip_coverage) side_imp;
  uvm_analysis_imp_cov_dma  #(i2c_dma_item, i2c_ip_coverage)     dma_imp;

  i2c_ip_env_cfg cfg;
  side_vif_t     side_vif;

  // sampled values
  bit [12:0]       cmd;
  i2c_ip_msg_info  mi;
  int unsigned     len;
  bit              any_nack;
  int              intr_bit;
  bit [1:0]        trx_tag;
  bit              crx_last;
  i2c_dma_item     dh;
  bit [1:0]        dma_sel;
  bit              dma_end_en;      // DMA_CTRL.END_EN mirrored from the APB writes (written only while unlocked)
  bit [11:0]       off;
  bit              apb_wr, apb_err;
  bit [3:0]        apb_strb;
  int unsigned     fifo_depth;

  covergroup cg_cmd;
    option.per_instance = 1;
    cp_start : coverpoint cmd[C_START];
    cp_stop  : coverpoint cmd[C_STOP];
    cp_read  : coverpoint cmd[C_READ];
    cp_nack  : coverpoint cmd[C_NACK];
    cp_nakok : coverpoint cmd[C_NAKOK];
    cp_kind  : coverpoint {cmd[C_NAKOK], cmd[C_NACK], cmd[C_READ], cmd[C_STOP], cmd[C_START]} {
      bins start_w      = {5'b00001};
      bins start_stop   = {5'b00011};
      bins start_nakok  = {5'b10001};
      bins wr           = {5'b00000};
      bins wr_stop      = {5'b00010};
      bins wr_nakok     = {5'b10000};
      bins wr_stop_nak  = {5'b10010};
      bins rd_ack       = {5'b00100};
      bins rd_nack      = {5'b01100};
      bins rd_stop      = {5'b00110, 5'b01110};
      bins illegal_mix  = {5'b00101, 5'b00111, 5'b01101};
    }
  endgroup

  covergroup cg_ctrl;
    option.per_instance = 1;
    cp_dir    : coverpoint mi.m.read;
    cp_len    : coverpoint len { bins none = {0}; bins one = {1}; bins few = {[2:4]}; bins many = {[5:15]}; bins huge = {[16:$]}; }
    cp_aack   : coverpoint mi.m.addr_ack;
    cp_rs     : coverpoint mi.m.repeated_start;
    cp_stop   : coverpoint mi.m.stop;
    cp_nack   : coverpoint any_nack;
    cp_speed  : coverpoint mi.speed;
    cp_hs     : coverpoint mi.hs;
    cp_prefix : coverpoint mi.ctrl_prefix;
    x_dir_len   : cross cp_dir, cp_len;
    x_dir_rs    : cross cp_dir, cp_rs;
    x_dir_speed : cross cp_dir, cp_speed;
    x_dir_aack  : cross cp_dir, cp_aack;
  endgroup

  covergroup cg_tgt;
    option.per_instance = 1;
    cp_kind : coverpoint mi.kind;
    cp_dir  : coverpoint mi.m.read;
    cp_ack  : coverpoint mi.dut_ack;
    cp_rs   : coverpoint mi.m.repeated_start;
    cp_len  : coverpoint len { bins none = {0}; bins one = {1}; bins few = {[2:4]}; bins many = {[5:15]}; bins huge = {[16:$]}; }
    cp_speed: coverpoint mi.speed;
    x_own_dir : cross cp_kind, cp_dir {
      ignore_bins na = binsof(cp_kind) intersect {TK_GC, TK_GC_OFF, TK_SB, TK_DEVW, TK_DEVR};
    }
    x_own_rs  : cross cp_kind, cp_rs { ignore_bins na = !binsof(cp_kind) intersect {TK_OWN0, TK_OWN1}; }
    x_dir_len : cross cp_dir, cp_len { ignore_bins na = binsof(cp_len) intersect {0}; }
  endgroup

  covergroup cg_trx;
    option.per_instance = 1;
    cp_tag : coverpoint trx_tag { bins data = {2'b00}; bins start = {2'b01}; bins stop = {2'b10}; bins rstart = {2'b11}; }
  endgroup

  covergroup cg_crx;
    option.per_instance = 1;
    cp_last : coverpoint crx_last;
  endgroup

  covergroup cg_intr;
    option.per_instance = 1;
    cp_bit : coverpoint intr_bit { bins b[] = {[0:21]}; }
  endgroup

  covergroup cg_dma;
    option.per_instance = 1;
    cp_sel  : coverpoint dma_sel { bins cmd = {0}; bins crx = {1}; bins ttx = {2}; bins trx = {3}; }
    cp_req  : coverpoint dh.req_type;
    cp_ack  : coverpoint dh.ack_type;
    x_sel_req : cross cp_sel, cp_req {
      ignore_bins tx_last = binsof(cp_sel) intersect {0, 2} && binsof(cp_req) intersect {DMA_LAST_SINGLE, DMA_LAST_BLOCK};
    }
    x_req_ack : cross cp_req, cp_ack {
      ignore_bins rsvd = binsof(cp_ack) intersect {DMA_ACK_RSVD} && !binsof(cp_req) intersect {DMA_SINGLE, DMA_BLOCK};
    }
  endgroup

  covergroup cg_dma_ev;
    option.per_instance = 1;
    cp_kind : coverpoint dh.kind;
    cp_end  : coverpoint dh.kind iff (dh.kind inside {DMA_EV_END_HS, DMA_EV_END_PULSE}) {
      bins hs = {DMA_EV_END_HS}; bins pulse = {DMA_EV_END_PULSE};
    }
    cp_end_en : coverpoint dma_end_en iff (dh.kind inside {DMA_EV_END_HS, DMA_EV_END_PULSE});
    cp_pend   : coverpoint dh.req_pending iff (dh.kind inside {DMA_EV_END_HS, DMA_EV_END_PULSE});
    x_end : cross cp_end, cp_end_en;
    x_end_pend : cross cp_end_en, cp_pend;
  endgroup

  covergroup cg_apb;
    option.per_instance = 1;
    cp_off : coverpoint off {
      bins ctrl = {R_CTRL}; bins status = {R_STATUS}; bins intr[] = {R_INTR_STAT, R_INTR_EN, R_INTR_TEST};
      bins fifo[] = {R_FIFO_CTRL, R_FIFO_STAT, R_CMD, R_CRXDATA, R_TTXDATA, R_TRXDATA};
      bins tgt[] = {R_TADDR, R_TCFG, R_TSTAT}; bins hscfg = {R_HSCFG}; bins cstat = {R_CSTAT};
      bins timing[] = {R_TIM_SCL, R_TIM_STA, R_TIM_STO, R_TIM_DAT, R_HS_TIM_SCL, R_HS_TIM_STA, R_HS_TIM_STO, R_HS_TIM_DAT, R_FILTER, R_TIMEOUT, R_BUSIDLE}; bins devid = {R_DEVID}; bins busmon = {R_BUSMON};
      bins dma[] = {R_DMA_CTRL, R_DMA_BLK, R_DMA_STAT}; bins id[] = {R_PARAM, R_VERSION};
      bins unmapped = {['h080:'h0F4], ['h100:'hFFC]};
    }
    cp_wr   : coverpoint apb_wr;
    cp_err  : coverpoint apb_err;
    cp_strb : coverpoint apb_strb iff (apb_wr) { bins none = {0}; bins full = {4'hF}; bins lane[] = {1, 2, 4, 8}; bins other = default; }
    x_off_wr : cross cp_off, cp_wr;
    x_err    : cross cp_off, cp_err {
      bins unmapped_err = binsof(cp_off.unmapped) && binsof(cp_err) intersect {1};
      bins fifo_err     = binsof(cp_off.fifo) && binsof(cp_err) intersect {1};
      ignore_bins rest  = !binsof(cp_off.unmapped) && !binsof(cp_off.fifo);
      ignore_bins never = binsof(cp_off.unmapped) && binsof(cp_err) intersect {0};   // unmapped always errors (3.7)
    }
  endgroup

  covergroup cg_depth;
    option.per_instance = 1;
    cp_depth : coverpoint fifo_depth { bins d4 = {4}; bins d8 = {8}; bins d16 = {16}; bins d32 = {32}; }
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_cmd = new(); cg_ctrl = new(); cg_tgt = new(); cg_trx = new(); cg_crx = new();
    cg_intr = new(); cg_dma = new(); cg_dma_ev = new(); cg_apb = new(); cg_depth = new();
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    apb_imp  = new("apb_imp", this);
    msg_imp  = new("msg_imp", this);
    side_imp = new("side_imp", this);
    dma_imp  = new("dma_imp", this);
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    fifo_depth = cfg.p.fifo_depth;
    cg_depth.sample();
  endfunction

  function void write_cov_apb(apb_item t);
    off = t.addr[11:0] & 12'hFFC; apb_wr = t.write; apb_err = t.slverr; apb_strb = t.strb;
    cg_apb.sample();
    if (t.slverr) return;
    if (t.write && off == R_DMA_CTRL && t.strb[0] && side_vif.d_dma_state == 0) dma_end_en = t.data[2];
    if (t.write && off == R_CMD && t.strb != 0) begin cmd = t.strobed_data(); cg_cmd.sample(); end
    if (!t.write && off == R_TRXDATA) begin trx_tag = t.data[9:8]; cg_trx.sample(); end
    if (!t.write && off == R_CRXDATA) begin crx_last = t.data[8]; cg_crx.sample(); end
  endfunction

  function void write_cov_msg(i2c_ip_msg_info t);
    mi = t;
    len = t.m.data.size();
    any_nack = 0;
    foreach (t.m.acks[i]) if (!t.m.acks[i] && !(t.m.read && i == t.m.acks.size() - 1)) any_nack = 1;
    if (t.by_ctrl) cg_ctrl.sample();
    else cg_tgt.sample();
  endfunction

  function void write_cov_side(i2c_ip_side_item t);
    if (t.kind != SIDE_INTR) return;
    intr_bit = t.idx;
    cg_intr.sample();
  endfunction

  function void write_cov_dma(i2c_dma_item t);
    dh = t;
    dma_sel = side_vif.d_dma_sel;
    if (t.kind == DMA_EV_HS) cg_dma.sample();
    cg_dma_ev.sample();
  endfunction
endclass : i2c_ip_coverage
