//-----------------------------------------------------------------------------
// i2c_regs (spec 3.6, 3.7, 4.8, 5)
//   APB4 slave interface and CSR file of the I2C IP.
//   - zero wait state (PREADY = 1 at top level), PSLVERR on unmapped access
//     and on write-full / read-empty FIFO data registers
//   - interrupt status / enable / test, 4 registered IRQ outputs
//   Only the defined (non-reserved) bits are stored. One always block per
//   register (word address); W1SC / W1C side effects next to their register.
//-----------------------------------------------------------------------------
module i2c_regs
  import apb_i2c_pkg::*;
#(
  parameter int          FIFO_DEPTH = 16,
  parameter logic [23:0] DEVID_RST  = 24'h000000,
  parameter logic [31:0] VERSION    = 32'h0002_0200
) (
  input  logic             PCLK,
  input  logic             PRESETn,
  // APB4
  input  logic             PSEL,
  input  logic             PENABLE,
  input  logic             PWRITE,
  input  logic [11:0]      PADDR,
  input  logic [31:0]      PWDATA,
  input  logic [3:0]       PSTRB,
  output logic [31:0]      PRDATA,
  output logic             PSLVERR,
  // CTRL
  output logic             cfg_en,
  output logic             cfg_ctrl_en,
  output logic             cfg_tgt_en,
  output logic             cfg_hs_en,
  output logic             cfg_sb_en,
  output logic             abort_req,
  output logic             busclr_req,
  output logic             swrst,
  output logic             flush_cmd,
  output logic             flush_crx,
  output logic             flush_ttx,
  output logic             flush_trx,
  // FIFO thresholds (raw register values)
  output logic [5:0]       thr_cmd,
  output logic [5:0]       thr_crx,
  output logic [5:0]       thr_ttx,
  output logic [5:0]       thr_trx,
  // target configuration
  output logic [6:0]       taddr0,
  output logic             taddr0_en,
  output logic [6:0]       taddr1,
  output logic             taddr1_en,
  output logic             gc_en,
  output logic             nostretch,
  output logic             tnack,
  output logic             devid_en,
  output logic [2:0]       mcode,
  // timing (raw register values)
  output i2c_timing_t      tim_fs,
  output i2c_timing_t      tim_hs,
  output logic [15:0]      fs_tbuf,
  output logic [7:0]       spklen_fs,
  output logic [7:0]       spklen_hs,
  output logic             tout_en,
  output logic [23:0]      tout_val,
  output logic [15:0]      idle_val,
  output logic [23:0]      devid,
  output logic             ovrd_en,
  output logic             scl_ovrd,
  output logic             sda_ovrd,
  output logic             dma_en,
  output logic             dma_pfc,
  output logic [1:0]       dma_sel,     // 0 CMD, 1 CRX, 2 TTX, 3 TRX
  output logic [5:0]       dma_blk,
  output logic             dma_end_en,  // DMA_CTRL.END_EN
  output logic             dma_clr,     // DMA_CTRL.CLR write pulse: withdraw the pending request
  // FIFO access
  output logic             cmd_push,
  output logic [CMD_W-1:0] cmd_wdata,
  input  logic             cmd_full,
  input  logic             cmd_empty,
  output logic             crx_pop,
  input  logic [8:0]       crx_rdata,
  input  logic             crx_full,
  input  logic             crx_empty,
  output logic             ttx_push,
  output logic [7:0]       ttx_wdata,
  input  logic             ttx_full,
  input  logic             ttx_empty,
  output logic             trx_pop,
  input  logic [9:0]       trx_rdata,
  input  logic             trx_full,
  input  logic             trx_empty,
  input  logic [5:0]       cmd_lvl,
  input  logic [5:0]       crx_lvl,
  input  logic [5:0]       ttx_lvl,
  input  logic [5:0]       trx_lvl,
  // status
  input  logic             st_bus_busy,
  input  logic             st_ctrl_active,
  input  logic             st_ctrl_hold,
  input  logic             st_tgt_active,
  input  logic             st_tgt_rw,
  input  logic             st_tgt_hold,
  input  logic             st_hs_active,
  input  logic             st_busclr,
  input  logic             mon_scl,
  input  logic             mon_sda,
  input  logic             mon_scl_oe,
  input  logic             mon_sda_oe,
  input  logic [10:0]      tstat,
  // interrupt sources
  input  logic [3:0]       lvl_irq,     // {TTX_THR, TRX_THR, CRX_THR, CMD_THR}
  input  logic [IR_W-1:0]  hw_ev,       // event pulses at INTR_STAT bit positions
  input  logic [6:0]       cstat_set,
  input  logic             abort_clr,
  input  logic             busclr_clr,
  input  logic             cmd_dma_clr,
  // DMA channel status
  input  logic             dma_req,
  input  logic             dma_busy,    // handshake not IDLE
  input  logic             dma_done,
  input  logic             dma_deny,
  input  logic             dma_end,     // end event from the DMAC (dma_end_req_i)
  output logic [IR_W-1:0]  intr_stat,   // raw INTR_STAT (debug)
  // interrupts
  output logic             irq_o,
  output logic             irq_ctrl_o,
  output logic             irq_tgt_o,
  output logic             irq_err_o
);

  //---------------------------------------------------------------------------
  // Address map (word offsets)
  //---------------------------------------------------------------------------
  localparam logic [9:0] A_CTRL      = 10'h000;
  localparam logic [9:0] A_STATUS    = 10'h001;
  localparam logic [9:0] A_INTR_STAT = 10'h002;
  localparam logic [9:0] A_INTR_EN   = 10'h003;
  localparam logic [9:0] A_INTR_TEST = 10'h004;
  localparam logic [9:0] A_FIFO_CTRL = 10'h005;
  localparam logic [9:0] A_FIFO_STAT = 10'h006;
  localparam logic [9:0] A_CMD       = 10'h007;
  localparam logic [9:0] A_CRXDATA   = 10'h008;
  localparam logic [9:0] A_TTXDATA   = 10'h009;
  localparam logic [9:0] A_TRXDATA   = 10'h00A;
  localparam logic [9:0] A_TADDR     = 10'h00B;
  localparam logic [9:0] A_TCFG      = 10'h00C;
  localparam logic [9:0] A_HSCFG     = 10'h00D;
  localparam logic [9:0] A_CSTAT     = 10'h00E;
  localparam logic [9:0] A_TSTAT     = 10'h00F;
  localparam logic [9:0] A_TIM_SCL   = 10'h010;
  localparam logic [9:0] A_TIM_STA   = 10'h011;
  localparam logic [9:0] A_TIM_STO   = 10'h012;
  localparam logic [9:0] A_TIM_DAT   = 10'h013;
  localparam logic [9:0] A_HS_SCL    = 10'h014;
  localparam logic [9:0] A_HS_STA    = 10'h015;
  localparam logic [9:0] A_HS_STO    = 10'h016;
  localparam logic [9:0] A_HS_DAT    = 10'h017;
  localparam logic [9:0] A_FILTER    = 10'h018;
  localparam logic [9:0] A_TIMEOUT   = 10'h019;
  localparam logic [9:0] A_BUSIDLE   = 10'h01A;
  localparam logic [9:0] A_DEVID     = 10'h01B;
  localparam logic [9:0] A_BUSMON    = 10'h01C;
  localparam logic [9:0] A_DMA_CTRL  = 10'h01D;
  localparam logic [9:0] A_DMA_BLK   = 10'h01E;
  localparam logic [9:0] A_DMA_STAT  = 10'h01F;
  localparam logic [9:0] A_PARAM     = 10'h03E;
  localparam logic [9:0] A_VERSION   = 10'h03F;

  //---------------------------------------------------------------------------
  // APB decode
  //---------------------------------------------------------------------------
  logic [9:0]  word_addr;
  logic        apb_access;
  logic        apb_write;
  logic        apb_read;
  logic        addr_mapped;
  logic [31:0] wdata_masked;   // PWDATA with unstrobed byte lanes = 0
  logic        any_strb;

  assign word_addr    = PADDR[11:2];
  assign apb_access   = PSEL & PENABLE;
  assign apb_write    = apb_access &  PWRITE;
  assign apb_read     = apb_access & ~PWRITE;
  assign wdata_masked = PWDATA & {{8{PSTRB[3]}}, {8{PSTRB[2]}}, {8{PSTRB[1]}}, {8{PSTRB[0]}}};
  assign any_strb     = |PSTRB;

  always_comb begin : p_addr_mapped
    unique case (word_addr)
      A_CTRL, A_STATUS, A_INTR_STAT, A_INTR_EN, A_INTR_TEST, A_FIFO_CTRL,
      A_FIFO_STAT, A_CMD, A_CRXDATA, A_TTXDATA, A_TRXDATA, A_TADDR, A_TCFG,
      A_HSCFG, A_CSTAT, A_TSTAT, A_TIM_SCL, A_TIM_STA, A_TIM_STO, A_TIM_DAT,
      A_HS_SCL, A_HS_STA, A_HS_STO, A_HS_DAT, A_FILTER, A_TIMEOUT, A_BUSIDLE,
      A_DEVID, A_BUSMON, A_DMA_CTRL, A_DMA_BLK, A_DMA_STAT,
      A_PARAM, A_VERSION: addr_mapped = 1'b1;
      default:            addr_mapped = 1'b0;
    endcase
  end

  // one-hot write strobe per register word (all registers are below 0x100)
  logic [63:0] wr_hit;
  assign wr_hit = (apb_write && (word_addr[9:6] == 4'd0)) ? (64'd1 << word_addr[5:0]) : 64'd0;

  // byte-lane update of a 16-bit field
  function automatic logic [15:0] wr16(input logic [15:0] old, input logic [15:0] d,
                                       input logic [1:0] strb);
    return {strb[1] ? d[15:8] : old[15:8], strb[0] ? d[7:0] : old[7:0]};
  endfunction

  //---------------------------------------------------------------------------
  // FIFO data registers (CMD, TTXDATA push; CRXDATA, TRXDATA pop) and PSLVERR
  //---------------------------------------------------------------------------
  logic wr_cmd, wr_ttx, rd_crx, rd_trx, fifo_err;

  assign wr_cmd    = wr_hit[A_CMD]     & any_strb;
  assign wr_ttx    = wr_hit[A_TTXDATA] & any_strb;
  assign rd_crx    = apb_read & (word_addr == A_CRXDATA);
  assign rd_trx    = apb_read & (word_addr == A_TRXDATA);

  assign cmd_push  = wr_cmd & ~cmd_full;
  assign cmd_wdata = wdata_masked[CMD_W-1:0];
  assign ttx_push  = wr_ttx & ~ttx_full;
  assign ttx_wdata = wdata_masked[7:0];
  assign crx_pop   = rd_crx & ~crx_empty;
  assign trx_pop   = rd_trx & ~trx_empty;

  assign fifo_err  = (wr_cmd & cmd_full) | (wr_ttx & ttx_full) |
                     (rd_crx & crx_empty) | (rd_trx & trx_empty);
  assign PSLVERR   = apb_access & (~addr_mapped | fifo_err);

  //---------------------------------------------------------------------------
  // CTRL: EN / CTRL_EN / TGT_EN / HS_EN / STARTBYTE_EN, W1SC controls.
  //   SWRST clears EN / CTRL_EN / TGT_EN / ABORT / BUSCLR (CTRL[4:3] keep the
  //   value written in the same access)
  //---------------------------------------------------------------------------
  logic       wr_ctrl;
  logic [4:0] r_ctrl;          // {STARTBYTE_EN, HS_EN, TGT_EN, CTRL_EN, EN}
  logic       r_abort;
  logic       r_busclr;

  assign wr_ctrl   = wr_hit[A_CTRL];
  assign swrst     = wr_ctrl & PSTRB[3] & PWDATA[31];
  assign flush_cmd = wr_ctrl & PSTRB[1] & PWDATA[8];
  assign flush_crx = wr_ctrl & PSTRB[1] & PWDATA[9];
  assign flush_ttx = wr_ctrl & PSTRB[1] & PWDATA[10];
  assign flush_trx = wr_ctrl & PSTRB[1] & PWDATA[11];

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_ctrl
    if (!PRESETn) begin
      r_ctrl <= 5'd0;
    end else begin
      if (wr_ctrl && PSTRB[0])      r_ctrl[4:3] <= PWDATA[4:3];
      if (swrst)                    r_ctrl[2:0] <= 3'd0;
      else if (wr_ctrl && PSTRB[0]) r_ctrl[2:0] <= PWDATA[2:0];
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_abort
    if (!PRESETn)                               r_abort <= 1'b0;
    else if (swrst)                             r_abort <= 1'b0;
    else if (wr_ctrl && PSTRB[2] && PWDATA[16]) r_abort <= 1'b1;
    else if (abort_clr)                         r_abort <= 1'b0;
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_busclr
    if (!PRESETn)                               r_busclr <= 1'b0;
    else if (swrst)                             r_busclr <= 1'b0;
    else if (wr_ctrl && PSTRB[2] && PWDATA[17]) r_busclr <= 1'b1;
    else if (busclr_clr)                        r_busclr <= 1'b0;
  end

  //---------------------------------------------------------------------------
  // INTR_STAT / INTR_EN / INTR_TEST: HW / INTR_TEST set wins over W1C
  //---------------------------------------------------------------------------
  logic [IR_W-1:0] r_intr_stat;     // event bits only (IR_EVENT_MASK)
  logic [IR_W-1:0] r_intr_en;
  logic [IR_W-1:0] intr_set;
  logic [IR_W-1:0] intr_w1c;
  logic [IR_W-1:0] intr_pend;

  assign intr_set = hw_ev | {dma_done, fifo_err, 20'd0} |
                    (wr_hit[A_INTR_TEST] ? wdata_masked[IR_W-1:0] : '0);
  assign intr_w1c = wr_hit[A_INTR_STAT] ? wdata_masked[IR_W-1:0] : '0;

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_intr_stat
    if (!PRESETn)   r_intr_stat <= '0;
    else if (swrst) r_intr_stat <= '0;
    else            r_intr_stat <= ((r_intr_stat & ~intr_w1c) | intr_set) & IR_EVENT_MASK;
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_intr_en
    if (!PRESETn) begin
      r_intr_en <= '0;
    end else if (wr_hit[A_INTR_EN]) begin
      if (PSTRB[0]) r_intr_en[7:0]   <= PWDATA[7:0];
      if (PSTRB[1]) r_intr_en[15:8]  <= PWDATA[15:8];
      if (PSTRB[2]) r_intr_en[21:16] <= PWDATA[21:16];
    end
  end

  assign intr_stat = r_intr_stat | {12'd0, lvl_irq[3:2], 6'd0, lvl_irq[1:0]};
  assign intr_pend = intr_stat & r_intr_en;

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_irq
    if (!PRESETn) begin
      irq_o      <= 1'b0;
      irq_ctrl_o <= 1'b0;
      irq_tgt_o  <= 1'b0;
      irq_err_o  <= 1'b0;
    end else begin
      irq_o      <= |intr_pend;
      irq_ctrl_o <= |intr_pend[7:0];
      irq_tgt_o  <= |intr_pend[15:8];
      irq_err_o  <= |intr_pend[20:16];
    end
  end

  //---------------------------------------------------------------------------
  // CSTAT (W1C, HW set wins)
  //---------------------------------------------------------------------------
  logic [6:0] r_cstat;

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_cstat
    if (!PRESETn)   r_cstat <= 7'd0;
    else if (swrst) r_cstat <= 7'd0;
    else            r_cstat <= (r_cstat & ~(wr_hit[A_CSTAT] ? wdata_masked[6:0] : 7'd0)) |
                               cstat_set;
  end

  //---------------------------------------------------------------------------
  // FIFO_CTRL thresholds
  //---------------------------------------------------------------------------
  logic [5:0] r_thr_cmd, r_thr_crx, r_thr_ttx, r_thr_trx;

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_fifo_ctrl
    if (!PRESETn) begin
      r_thr_cmd <= 6'd0;
      r_thr_crx <= 6'd0;
      r_thr_ttx <= 6'd0;
      r_thr_trx <= 6'd0;
    end else if (wr_hit[A_FIFO_CTRL]) begin
      if (PSTRB[0]) r_thr_cmd <= PWDATA[5:0];
      if (PSTRB[1]) r_thr_crx <= PWDATA[13:8];
      if (PSTRB[2]) r_thr_ttx <= PWDATA[21:16];
      if (PSTRB[3]) r_thr_trx <= PWDATA[29:24];
    end
  end

  //---------------------------------------------------------------------------
  // Target configuration: TADDR, TCFG, HSCFG, DEVID
  //---------------------------------------------------------------------------
  logic [15:0] r_taddr;         // {ADDR1_EN, ADDR1, ADDR0_EN, ADDR0}
  logic [3:0]  r_tcfg;          // {DEVID_EN, TNACK, NOSTRETCH, GC_EN}
  logic [2:0]  r_mcode;
  logic [23:0] r_devid;

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_taddr
    if (!PRESETn)             r_taddr <= 16'd0;
    else if (wr_hit[A_TADDR]) r_taddr <= wr16(r_taddr, PWDATA[15:0], PSTRB[1:0]);
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_tcfg
    if (!PRESETn)                        r_tcfg <= 4'd0;
    else if (wr_hit[A_TCFG] && PSTRB[0]) r_tcfg <= PWDATA[3:0];
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_hscfg
    if (!PRESETn)                         r_mcode <= 3'b001;
    else if (wr_hit[A_HSCFG] && PSTRB[0]) r_mcode <= PWDATA[2:0];
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_devid
    if (!PRESETn) begin
      r_devid <= DEVID_RST;
    end else if (wr_hit[A_DEVID]) begin
      if (PSTRB[0]) r_devid[7:0]   <= PWDATA[7:0];
      if (PSTRB[1]) r_devid[15:8]  <= PWDATA[15:8];
      if (PSTRB[2]) r_devid[23:16] <= PWDATA[23:16];
    end
  end

  //---------------------------------------------------------------------------
  // TIMEOUT (not locked) and BUSMON override
  //---------------------------------------------------------------------------
  logic        r_tout_en;
  logic [23:0] r_tout_val;
  logic [2:0]  r_busmon;        // {SDA_OVRD, SCL_OVRD, OVRD_EN}

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_timeout
    if (!PRESETn) begin
      r_tout_en  <= 1'b0;
      r_tout_val <= 24'h35_67E0;
    end else if (wr_hit[A_TIMEOUT]) begin
      if (PSTRB[0]) r_tout_val[7:0]   <= PWDATA[7:0];
      if (PSTRB[1]) r_tout_val[15:8]  <= PWDATA[15:8];
      if (PSTRB[2]) r_tout_val[23:16] <= PWDATA[23:16];
      if (PSTRB[3]) r_tout_en         <= PWDATA[31];
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_busmon
    if (!PRESETn)                          r_busmon <= 3'b110;
    else if (wr_hit[A_BUSMON] && PSTRB[1]) r_busmon <= PWDATA[10:8];
  end

  //---------------------------------------------------------------------------
  // Timing registers: writes ignored while CTRL.EN = 1 (one block per word)
  //---------------------------------------------------------------------------
  logic        tim_wr_en;
  logic [15:0] r_fs_scl_low, r_fs_scl_high, r_fs_thd_sta, r_fs_tsu_sta;
  logic [15:0] r_fs_tsu_sto, r_fs_thd_dat, r_fs_tsu_dat;
  logic [15:0] r_hs_scl_low, r_hs_scl_high, r_hs_thd_sta, r_hs_tsu_sta;
  logic [15:0] r_hs_tsu_sto, r_hs_thd_dat, r_hs_tsu_dat;
  logic [15:0] r_tbuf;
  logic [7:0]  r_spklen_fs, r_spklen_hs;
  logic [15:0] r_idle_val;

  assign tim_wr_en = ~r_ctrl[0];

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_tim_scl
    if (!PRESETn) begin
      r_fs_scl_low  <= 16'd500;
      r_fs_scl_high <= 16'd490;
    end else if (tim_wr_en && wr_hit[A_TIM_SCL]) begin
      r_fs_scl_low  <= wr16(r_fs_scl_low,  PWDATA[15:0],  PSTRB[1:0]);
      r_fs_scl_high <= wr16(r_fs_scl_high, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_tim_sta
    if (!PRESETn) begin
      r_fs_thd_sta <= 16'd400;
      r_fs_tsu_sta <= 16'd470;
    end else if (tim_wr_en && wr_hit[A_TIM_STA]) begin
      r_fs_thd_sta <= wr16(r_fs_thd_sta, PWDATA[15:0],  PSTRB[1:0]);
      r_fs_tsu_sta <= wr16(r_fs_tsu_sta, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_tim_sto
    if (!PRESETn) begin
      r_fs_tsu_sto <= 16'd400;
      r_tbuf           <= 16'd470;
    end else if (tim_wr_en && wr_hit[A_TIM_STO]) begin
      r_fs_tsu_sto <= wr16(r_fs_tsu_sto, PWDATA[15:0],  PSTRB[1:0]);
      r_tbuf           <= wr16(r_tbuf,           PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_tim_dat
    if (!PRESETn) begin
      r_fs_thd_dat <= 16'd30;
      r_fs_tsu_dat <= 16'd125;
    end else if (tim_wr_en && wr_hit[A_TIM_DAT]) begin
      r_fs_thd_dat <= wr16(r_fs_thd_dat, PWDATA[15:0],  PSTRB[1:0]);
      r_fs_tsu_dat <= wr16(r_fs_tsu_dat, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_hs_scl
    if (!PRESETn) begin
      r_hs_scl_low  <= 16'd15;
      r_hs_scl_high <= 16'd5;
    end else if (tim_wr_en && wr_hit[A_HS_SCL]) begin
      r_hs_scl_low  <= wr16(r_hs_scl_low,  PWDATA[15:0],  PSTRB[1:0]);
      r_hs_scl_high <= wr16(r_hs_scl_high, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_hs_sta
    if (!PRESETn) begin
      r_hs_thd_sta <= 16'd16;
      r_hs_tsu_sta <= 16'd16;
    end else if (tim_wr_en && wr_hit[A_HS_STA]) begin
      r_hs_thd_sta <= wr16(r_hs_thd_sta, PWDATA[15:0],  PSTRB[1:0]);
      r_hs_tsu_sta <= wr16(r_hs_tsu_sta, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_hs_sto
    if (!PRESETn)                           r_hs_tsu_sto <= 16'd16;
    else if (tim_wr_en && wr_hit[A_HS_STO]) r_hs_tsu_sto <= wr16(r_hs_tsu_sto, PWDATA[15:0], PSTRB[1:0]);
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_hs_dat
    if (!PRESETn) begin
      r_hs_thd_dat <= 16'd1;
      r_hs_tsu_dat <= 16'd9;
    end else if (tim_wr_en && wr_hit[A_HS_DAT]) begin
      r_hs_thd_dat <= wr16(r_hs_thd_dat, PWDATA[15:0],  PSTRB[1:0]);
      r_hs_tsu_dat <= wr16(r_hs_tsu_dat, PWDATA[31:16], PSTRB[3:2]);
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_filter
    if (!PRESETn) begin
      r_spklen_fs <= 8'd5;
      r_spklen_hs <= 8'd1;
    end else if (tim_wr_en && wr_hit[A_FILTER]) begin
      if (PSTRB[0]) r_spklen_fs <= PWDATA[7:0];
      if (PSTRB[1]) r_spklen_hs <= PWDATA[15:8];
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_busidle
    if (!PRESETn)                            r_idle_val <= 16'd1000;
    else if (tim_wr_en && wr_hit[A_BUSIDLE]) r_idle_val <= wr16(r_idle_val, PWDATA[15:0], PSTRB[1:0]);
  end

  //---------------------------------------------------------------------------
  // DMA_CTRL / DMA_BLK / DMA_STAT (one DMA channel, FIFO selected by SEL)
  //   EN is cleared by HW when the channel completes and, with SEL = CMD, on a
  //   controller abort (HW clear > SW write). SEL / PFC / END_EN / BLK writes
  //   are ignored while EN = 1 or a handshake is in progress. CLR is a write
  //   command (reads 0).
  //---------------------------------------------------------------------------
  logic       dma_cfg_wr_en;
  logic       wr_dma_ctrl;
  logic       r_dma_en;
  logic       r_dma_pfc;
  logic       r_dma_end_en;
  logic [1:0] r_dma_sel;
  logic [5:0] r_dma_blk;
  logic       r_dma_done;
  logic       r_dma_deny;
  logic       r_dma_end;

  assign dma_cfg_wr_en = ~r_dma_en & ~dma_busy;
  assign wr_dma_ctrl   = wr_hit[A_DMA_CTRL] & PSTRB[0];

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_dma_en
    if (!PRESETn) r_dma_en <= 1'b0;
    else if (swrst || dma_done || (cmd_dma_clr && (r_dma_sel == 2'd0)))
                                                        r_dma_en <= 1'b0;
    else if (wr_dma_ctrl) r_dma_en <= PWDATA[0];
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_dma_cfg
    if (!PRESETn) begin
      r_dma_pfc    <= 1'b0;
      r_dma_end_en <= 1'b0;
      r_dma_sel    <= 2'd0;
    end else if (wr_dma_ctrl && dma_cfg_wr_en) begin
      r_dma_pfc    <= PWDATA[1];
      r_dma_end_en <= PWDATA[2];
      r_dma_sel    <= PWDATA[5:4];
    end
  end

  assign dma_clr = wr_dma_ctrl & PWDATA[3];

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_dma_blk
    if (!PRESETn)                                            r_dma_blk <= 6'd0;
    else if (wr_hit[A_DMA_BLK] && PSTRB[0] && dma_cfg_wr_en) r_dma_blk <= PWDATA[5:0];
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_dma_stat
    if (!PRESETn) begin
      r_dma_done <= 1'b0;
      r_dma_deny <= 1'b0;
      r_dma_end  <= 1'b0;
    end else if (swrst) begin
      r_dma_done <= 1'b0;
      r_dma_deny <= 1'b0;
      r_dma_end  <= 1'b0;
    end else begin
      r_dma_done <= (r_dma_done & ~(wr_hit[A_DMA_STAT] & wdata_masked[8]))  | dma_done;
      r_dma_deny <= (r_dma_deny & ~(wr_hit[A_DMA_STAT] & wdata_masked[16])) | dma_deny;
      r_dma_end  <= (r_dma_end  & ~(wr_hit[A_DMA_STAT] & wdata_masked[24])) | dma_end;
    end
  end

  //---------------------------------------------------------------------------
  // Configuration outputs
  //---------------------------------------------------------------------------
  assign cfg_en      = r_ctrl[0];
  assign cfg_ctrl_en = r_ctrl[1];
  assign cfg_tgt_en  = r_ctrl[2];
  assign cfg_hs_en   = r_ctrl[3];
  assign cfg_sb_en   = r_ctrl[4];
  assign abort_req   = r_abort;
  assign busclr_req  = r_busclr;
  assign thr_cmd     = r_thr_cmd;
  assign thr_crx     = r_thr_crx;
  assign thr_ttx     = r_thr_ttx;
  assign thr_trx     = r_thr_trx;
  assign taddr0      = r_taddr[6:0];
  assign taddr0_en   = r_taddr[7];
  assign taddr1      = r_taddr[14:8];
  assign taddr1_en   = r_taddr[15];
  assign gc_en       = r_tcfg[0];
  assign nostretch   = r_tcfg[1];
  assign tnack       = r_tcfg[2];
  assign devid_en    = r_tcfg[3];
  assign mcode       = r_mcode;
  assign tim_fs      = '{scl_low: r_fs_scl_low, scl_high: r_fs_scl_high,
                         thd_sta: r_fs_thd_sta, tsu_sta:  r_fs_tsu_sta, tsu_sto: r_fs_tsu_sto,
                         thd_dat: r_fs_thd_dat, tsu_dat:  r_fs_tsu_dat};
  assign tim_hs      = '{scl_low: r_hs_scl_low, scl_high: r_hs_scl_high,
                         thd_sta: r_hs_thd_sta, tsu_sta:  r_hs_tsu_sta, tsu_sto: r_hs_tsu_sto,
                         thd_dat: r_hs_thd_dat, tsu_dat:  r_hs_tsu_dat};
  assign fs_tbuf     = r_tbuf;
  assign spklen_fs   = r_spklen_fs;
  assign spklen_hs   = r_spklen_hs;
  assign tout_en     = r_tout_en;
  assign tout_val    = r_tout_val;
  assign idle_val    = r_idle_val;
  assign devid       = r_devid;
  assign ovrd_en     = r_busmon[0];
  assign scl_ovrd    = r_busmon[1];
  assign sda_ovrd    = r_busmon[2];
  assign dma_en      = r_dma_en;
  assign dma_pfc     = r_dma_pfc;
  assign dma_sel     = r_dma_sel;
  assign dma_blk     = r_dma_blk;
  assign dma_end_en  = r_dma_end_en;

  //---------------------------------------------------------------------------
  // Read data (driven only during a read transfer to limit bus toggling)
  //---------------------------------------------------------------------------
  always_comb begin : p_prdata
    PRDATA = 32'd0;
    if (PSEL && !PWRITE) begin
      unique case (word_addr)
        A_CTRL:      PRDATA = {14'd0, r_busclr, r_abort, 11'd0, r_ctrl};
        A_STATUS:    PRDATA = {16'd0,
                               trx_empty, trx_full, ttx_empty, ttx_full,
                               crx_empty, crx_full, cmd_empty, cmd_full,
                               st_busclr, st_hs_active, st_tgt_hold, st_tgt_rw,
                               st_tgt_active, st_ctrl_hold, st_ctrl_active, st_bus_busy};
        A_INTR_STAT: PRDATA = {10'd0, intr_stat};
        A_INTR_EN:   PRDATA = {10'd0, r_intr_en};
        A_FIFO_CTRL: PRDATA = {2'd0, r_thr_trx, 2'd0, r_thr_ttx, 2'd0, r_thr_crx, 2'd0, r_thr_cmd};
        A_FIFO_STAT: PRDATA = {2'd0, trx_lvl, 2'd0, ttx_lvl, 2'd0, crx_lvl, 2'd0, cmd_lvl};
        A_CRXDATA:   PRDATA = crx_empty ? 32'd0 : {23'd0, crx_rdata};
        A_TRXDATA:   PRDATA = trx_empty ? 32'd0 : {22'd0, trx_rdata};
        A_TADDR:     PRDATA = {16'd0, r_taddr};
        A_TCFG:      PRDATA = {28'd0, r_tcfg};
        A_HSCFG:     PRDATA = {29'd0, r_mcode};
        A_CSTAT:     PRDATA = {25'd0, r_cstat};
        A_TSTAT:     PRDATA = {21'd0, tstat};
        A_TIM_SCL:   PRDATA = {r_fs_scl_high, r_fs_scl_low};
        A_TIM_STA:   PRDATA = {r_fs_tsu_sta,  r_fs_thd_sta};
        A_TIM_STO:   PRDATA = {r_tbuf,            r_fs_tsu_sto};
        A_TIM_DAT:   PRDATA = {r_fs_tsu_dat,  r_fs_thd_dat};
        A_HS_SCL:    PRDATA = {r_hs_scl_high, r_hs_scl_low};
        A_HS_STA:    PRDATA = {r_hs_tsu_sta,  r_hs_thd_sta};
        A_HS_STO:    PRDATA = {16'd0,             r_hs_tsu_sto};
        A_HS_DAT:    PRDATA = {r_hs_tsu_dat,  r_hs_thd_dat};
        A_FILTER:    PRDATA = {16'd0, r_spklen_hs, r_spklen_fs};
        A_TIMEOUT:   PRDATA = {r_tout_en, 7'd0, r_tout_val};
        A_BUSIDLE:   PRDATA = {16'd0, r_idle_val};
        A_DEVID:     PRDATA = {8'd0, r_devid};
        A_BUSMON:    PRDATA = {21'd0, r_busmon, 4'd0, mon_sda_oe, mon_scl_oe, mon_sda, mon_scl};
        A_DMA_CTRL:  PRDATA = {26'd0, r_dma_sel, 1'b0, r_dma_end_en, r_dma_pfc, r_dma_en};
        A_DMA_BLK:   PRDATA = {26'd0, r_dma_blk};
        A_DMA_STAT:  PRDATA = {7'd0, r_dma_end, 7'd0, r_dma_deny, 7'd0, r_dma_done, 6'd0, dma_busy, dma_req};
        A_PARAM:     PRDATA = {19'd0, 5'b11111, 8'(FIFO_DEPTH)};  // QCH, DMA_END, DMA, DEVID, HS
        A_VERSION:   PRDATA = VERSION;
        default:     PRDATA = 32'd0;     // WO / unmapped
      endcase
    end
  end

endmodule
