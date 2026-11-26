//-----------------------------------------------------------------------------
// apb_i2c
//   APB4 I2C Controller / Target IP - top level.
//
//   One DMA channel serving the FIFO selected by DMA_CTRL.SEL (0 CMD, 1 CRX,
//   2 TTX, 3 TRX): request handshake dma_req_o / dma_req_type_o / dma_ack_i /
//   dma_ack_type_i and end handshake dma_end_req_i / dma_end_ack_o (the trigger
//   input / trigger output pair of an Arm DMA-350 channel, spec 3.4).
//-----------------------------------------------------------------------------
module apb_i2c
  import apb_i2c_pkg::*;
#(
  parameter int          FIFO_DEPTH = 16,                // 4, 8, 16, 32
  parameter int          APB_AW     = 12,
  parameter logic [23:0] DEVID_RST  = 24'h000000,
  parameter logic [31:0] VERSION    = 32'h0002_0100
) (
  // APB4
  input  logic              PCLK,
  input  logic              PRESETn,
  input  logic              PSEL,
  input  logic              PENABLE,
  input  logic              PWRITE,
  input  logic [APB_AW-1:0] PADDR,
  input  logic [31:0]       PWDATA,
  input  logic [3:0]        PSTRB,
  input  logic [2:0]        PPROT,
  output logic [31:0]       PRDATA,
  output logic              PREADY,
  output logic              PSLVERR,
  // I2C pads
  input  logic              scl_i,
  output logic              scl_oe_o,
  input  logic              sda_i,
  output logic              sda_oe_o,
  output logic              scl_cs_en_o,
  output logic              hs_mode_o,
  // interrupts
  output logic              irq_o,
  output logic              irq_ctrl_o,
  output logic              irq_tgt_o,
  output logic              irq_err_o,
  // DMA request / acknowledge (4-phase handshake with type qualifiers)
  output logic              dma_req_o,
  output logic [1:0]        dma_req_type_o,
  input  logic              dma_ack_i,
  input  logic [1:0]        dma_ack_type_i,
  // DMA end of operation, DMAC -> IP (4-phase handshake or pulse)
  input  logic              dma_end_req_i,
  output logic              dma_end_ack_o,
  // status for SoC logic (registered, spec 3.5)
  output logic              debug_bus_busy_o,
  output logic              debug_ctrl_active_o,
  output logic              debug_ctrl_hold_o,
  output logic              debug_tgt_active_o,
  output logic              debug_tgt_hold_o,
  output logic [1:0]        debug_dma_sel_o,
  output logic [21:0]       debug_intr_stat_o
);

  //---------------------------------------------------------------------------
  // Parameter checks (elaboration time)
  //---------------------------------------------------------------------------
  if (!((FIFO_DEPTH == 4) || (FIFO_DEPTH == 8) || (FIFO_DEPTH == 16) || (FIFO_DEPTH == 32))) begin : g_bad_depth
    $fatal(1, "apb_i2c: FIFO_DEPTH must be 4, 8, 16 or 32");
  end
  if (APB_AW != 12) begin : g_bad_aw
    $fatal(1, "apb_i2c: APB_AW must be 12");
  end

  localparam logic [5:0] DEPTH6 = 6'(FIFO_DEPTH);

  //---------------------------------------------------------------------------
  // Signals
  //---------------------------------------------------------------------------
  // configuration (CSR)
  logic        cfg_en, cfg_ctrl_en, cfg_tgt_en, cfg_hs_en, cfg_sb_en;
  logic        abort_req, busclr_req, swrst;
  logic        sw_flush_cmd, sw_flush_crx, sw_flush_ttx, sw_flush_trx;
  logic [5:0]  thr_cmd, thr_crx, thr_ttx, thr_trx;
  logic [6:0]  taddr0, taddr1;
  logic        taddr0_en, taddr1_en;
  logic        gc_en, nostretch, tnack, devid_en;
  logic [2:0]  mcode;
  i2c_timing_t tim_fs, tim_hs;
  logic [15:0] fs_tbuf;
  logic [7:0]  spklen_fs, spklen_hs;
  logic        tout_en;
  logic [23:0] tout_val;
  logic [15:0] idle_val;
  logic [23:0] devid;
  logic        ovrd_en, scl_ovrd, sda_ovrd;
  logic        dma_en, dma_pfc_cfg, dma_end_en_cfg, dma_clr;
  logic [1:0]  dma_sel;
  logic [5:0]  dma_blk_cfg;
  logic        core_en;

  // FIFOs
  logic        cmd_push, cmd_pop, cmd_full, cmd_empty;
  logic [CMD_W-1:0] cmd_wdata, cmd_rdata;
  logic [5:0]  cmd_lvl;
  logic        crx_push, crx_pop, crx_full, crx_empty;
  logic [8:0]  crx_wdata, crx_rdata;
  logic [5:0]  crx_lvl, crx_last_pos;
  logic        ttx_push, ttx_pop, ttx_full, ttx_empty;
  logic [7:0]  ttx_wdata, ttx_rdata;
  logic [5:0]  ttx_lvl;
  logic        trx_push, trx_pop, trx_full, trx_empty;
  logic [9:0]  trx_wdata, trx_rdata;
  logic [5:0]  trx_lvl, trx_stop_pos;
  logic [3:0]  lvl_irq;

  // bus
  logic        scl_f, sda_f;
  logic        scl_rise, scl_fall, sda_rise, sda_fall;
  logic        start_det, stop_det, bus_busy, bus_free, hs_active, hs_det;
  logic        bus_err, scl_tout;

  // controller
  logic        ctrl_cmd_flush;
  logic        ctrl_scl_oe, ctrl_sda_oe, ctrl_cs_en, ctrl_owner, ctrl_busclr, ctrl_hold;
  logic        ctrl_abort_clr, ctrl_busclr_clr;
  logic        ctrl_ev_cmpl, ctrl_ev_nack, ctrl_ev_arb, ctrl_ev_cmderr, ctrl_ev_stall;
  logic        ctrl_ev_bcdone;
  logic [6:0]  ctrl_cstat_set;

  // target
  logic        tgt_scl_oe, tgt_sda_oe, tgt_active, tgt_rw, tgt_hold;
  logic        tgt_ev_tx_req, tgt_ev_taddr, tgt_ev_tstop, tgt_ev_ttx_done, tgt_ev_gc_swrst;
  logic        tgt_ev_trx_ovf, tgt_ev_ttx_unf;
  logic [10:0] tstat;

  // DMA
  logic [5:0]  dma_avail, dma_lpos;
  logic        dma_done, dma_deny, dma_end_ev;
  dma_state_t  dma_state;

  // misc
  logic [IR_W-1:0] hw_ev;
  logic [IR_W-1:0] intr_stat;
  logic        scl_oe_nxt, sda_oe_nxt, cs_en_nxt;

  function automatic logic [5:0] thr_clamp(input logic [5:0] v);
    return (v >= DEPTH6) ? (DEPTH6 - 6'd1) : v;
  endfunction

  //---------------------------------------------------------------------------
  // APB / CSR
  //---------------------------------------------------------------------------
  assign PREADY = 1'b1;

  i2c_regs #(
    .FIFO_DEPTH (FIFO_DEPTH),
    .DEVID_RST  (DEVID_RST),
    .VERSION    (VERSION)
  ) u_regs (
    .PCLK           (PCLK),
    .PRESETn        (PRESETn),
    .PSEL           (PSEL),
    .PENABLE        (PENABLE),
    .PWRITE         (PWRITE),
    .PADDR          (PADDR),
    .PWDATA         (PWDATA),
    .PSTRB          (PSTRB),
    .PRDATA         (PRDATA),
    .PSLVERR        (PSLVERR),
    .cfg_en         (cfg_en),
    .cfg_ctrl_en    (cfg_ctrl_en),
    .cfg_tgt_en     (cfg_tgt_en),
    .cfg_hs_en      (cfg_hs_en),
    .cfg_sb_en      (cfg_sb_en),
    .abort_req      (abort_req),
    .busclr_req     (busclr_req),
    .swrst          (swrst),
    .flush_cmd      (sw_flush_cmd),
    .flush_crx      (sw_flush_crx),
    .flush_ttx      (sw_flush_ttx),
    .flush_trx      (sw_flush_trx),
    .thr_cmd        (thr_cmd),
    .thr_crx        (thr_crx),
    .thr_ttx        (thr_ttx),
    .thr_trx        (thr_trx),
    .taddr0         (taddr0),
    .taddr0_en      (taddr0_en),
    .taddr1         (taddr1),
    .taddr1_en      (taddr1_en),
    .gc_en          (gc_en),
    .nostretch      (nostretch),
    .tnack          (tnack),
    .devid_en       (devid_en),
    .mcode          (mcode),
    .tim_fs         (tim_fs),
    .tim_hs         (tim_hs),
    .fs_tbuf        (fs_tbuf),
    .spklen_fs      (spklen_fs),
    .spklen_hs      (spklen_hs),
    .tout_en        (tout_en),
    .tout_val       (tout_val),
    .idle_val       (idle_val),
    .devid          (devid),
    .ovrd_en        (ovrd_en),
    .scl_ovrd       (scl_ovrd),
    .sda_ovrd       (sda_ovrd),
    .dma_en         (dma_en),
    .dma_pfc        (dma_pfc_cfg),
    .dma_sel        (dma_sel),
    .dma_blk        (dma_blk_cfg),
    .dma_end_en     (dma_end_en_cfg),
    .dma_clr        (dma_clr),
    .cmd_push       (cmd_push),
    .cmd_wdata      (cmd_wdata),
    .cmd_full       (cmd_full),
    .cmd_empty      (cmd_empty),
    .crx_pop        (crx_pop),
    .crx_rdata      (crx_rdata),
    .crx_full       (crx_full),
    .crx_empty      (crx_empty),
    .ttx_push       (ttx_push),
    .ttx_wdata      (ttx_wdata),
    .ttx_full       (ttx_full),
    .ttx_empty      (ttx_empty),
    .trx_pop        (trx_pop),
    .trx_rdata      (trx_rdata),
    .trx_full       (trx_full),
    .trx_empty      (trx_empty),
    .cmd_lvl        (cmd_lvl),
    .crx_lvl        (crx_lvl),
    .ttx_lvl        (ttx_lvl),
    .trx_lvl        (trx_lvl),
    .st_bus_busy    (bus_busy),
    .st_ctrl_active (ctrl_owner | ctrl_busclr),
    .st_ctrl_hold   (ctrl_hold),
    .st_tgt_active  (tgt_active),
    .st_tgt_rw      (tgt_rw),
    .st_tgt_hold    (tgt_hold),
    .st_hs_active   (hs_active),
    .st_busclr      (ctrl_busclr),
    .mon_scl        (scl_f),
    .mon_sda        (sda_f),
    .mon_scl_oe     (scl_oe_o),
    .mon_sda_oe     (sda_oe_o),
    .tstat          (tstat),
    .lvl_irq        (lvl_irq),
    .hw_ev          (hw_ev),
    .cstat_set      (ctrl_cstat_set),
    .abort_clr      (ctrl_abort_clr),
    .busclr_clr     (ctrl_busclr_clr),
    .cmd_dma_clr    (ctrl_cmd_flush),
    .dma_req        (dma_req_o),
    .dma_busy       (dma_state != D_IDLE),
    .dma_done       (dma_done),
    .dma_deny       (dma_deny),
    .dma_end        (dma_end_ev),
    .intr_stat      (intr_stat),
    .irq_o          (irq_o),
    .irq_ctrl_o     (irq_ctrl_o),
    .irq_tgt_o      (irq_tgt_o),
    .irq_err_o      (irq_err_o)
  );

  // core enable: CTRL.EN and not in the software reset cycle
  assign core_en = cfg_en & ~swrst;

  //---------------------------------------------------------------------------
  // FIFOs
  //   CRX entry : {LAST, data}   mark = LAST     (final byte of a read)
  //   TRX entry : {TAG, data}    mark = TAG STOP (end of a transaction)
  //---------------------------------------------------------------------------
  i2c_sync_fifo #(.WIDTH(CMD_W), .DEPTH(FIFO_DEPTH)) u_cmd_fifo (
    .clk   (PCLK),      .rst_n (PRESETn),
    .flush (sw_flush_cmd | swrst | ctrl_cmd_flush),
    .push  (cmd_push),  .wdata (cmd_wdata), .wmark (1'b0),
    .pop   (cmd_pop),   .rdata (cmd_rdata),
    .level (cmd_lvl),   .full  (cmd_full),  .empty (cmd_empty), .mpos ()
  );

  i2c_sync_fifo #(.WIDTH(9), .DEPTH(FIFO_DEPTH), .MARK(1'b1)) u_crx_fifo (
    .clk   (PCLK),      .rst_n (PRESETn),
    .flush (sw_flush_crx | swrst),
    .push  (crx_push),  .wdata (crx_wdata), .wmark (crx_wdata[8]),
    .pop   (crx_pop),   .rdata (crx_rdata),
    .level (crx_lvl),   .full  (crx_full),  .empty (crx_empty), .mpos (crx_last_pos)
  );

  i2c_sync_fifo #(.WIDTH(8), .DEPTH(FIFO_DEPTH)) u_ttx_fifo (
    .clk   (PCLK),      .rst_n (PRESETn),
    .flush (sw_flush_ttx | swrst),
    .push  (ttx_push),  .wdata (ttx_wdata), .wmark (1'b0),
    .pop   (ttx_pop),   .rdata (ttx_rdata),
    .level (ttx_lvl),   .full  (ttx_full),  .empty (ttx_empty), .mpos ()
  );

  i2c_sync_fifo #(.WIDTH(10), .DEPTH(FIFO_DEPTH), .MARK(1'b1)) u_trx_fifo (
    .clk   (PCLK),      .rst_n (PRESETn),
    .flush (sw_flush_trx | swrst),
    .push  (trx_push),  .wdata (trx_wdata), .wmark (trx_wdata[9:8] == TAG_STOP),
    .pop   (trx_pop),   .rdata (trx_rdata),
    .level (trx_lvl),   .full  (trx_full),  .empty (trx_empty), .mpos (trx_stop_pos)
  );

  // threshold (level) interrupts {TTX_THR, TRX_THR, CRX_THR, CMD_THR}
  assign lvl_irq = {(ttx_lvl <= thr_clamp(thr_ttx)),
                    (trx_lvl >  thr_clamp(thr_trx)),
                    (crx_lvl >  thr_clamp(thr_crx)),
                    (cmd_lvl <= thr_clamp(thr_cmd))};

  //---------------------------------------------------------------------------
  // Input conditioning and bus monitor
  //---------------------------------------------------------------------------
  i2c_in_filter u_scl_filt (
    .clk    (PCLK),     .rst_n  (PRESETn), .din  (scl_i),
    .spklen (hs_active ? spklen_hs : spklen_fs),
    .dout   (scl_f),    .rise   (scl_rise), .fall (scl_fall)
  );

  i2c_in_filter u_sda_filt (
    .clk    (PCLK),     .rst_n  (PRESETn), .din  (sda_i),
    .spklen (hs_active ? spklen_hs : spklen_fs),
    .dout   (sda_f),    .rise   (sda_rise), .fall (sda_fall)
  );

  i2c_bus_mon u_bus_mon (
    .clk           (PCLK),
    .rst_n         (PRESETn),
    .en            (core_en),
    .busclr_active (ctrl_busclr),
    .scl_f         (scl_f),
    .sda_f         (sda_f),
    .scl_rise      (scl_rise),
    .scl_fall      (scl_fall),
    .sda_rise      (sda_rise),
    .sda_fall      (sda_fall),
    .tbuf          (fs_tbuf),
    .idle_val      (idle_val),
    .tout_en       (tout_en),
    .tout_val      (tout_val),
    .start_det     (start_det),
    .stop_det      (stop_det),
    .bus_busy      (bus_busy),
    .bus_free      (bus_free),
    .hs_active     (hs_active),
    .hs_det        (hs_det),
    .bus_err       (bus_err),
    .scl_tout      (scl_tout)
  );

  //---------------------------------------------------------------------------
  // Controller
  //---------------------------------------------------------------------------
  i2c_ctrl u_ctrl (
    .clk           (PCLK),
    .rst_n         (PRESETn),
    .en            (core_en),
    .ctrl_en       (cfg_ctrl_en),
    .hs_en         (cfg_hs_en),
    .sb_en         (cfg_sb_en),
    .mcode         (mcode),
    .abort_req     (abort_req),
    .busclr_req    (busclr_req),
    .cmd_empty     (cmd_empty),
    .cmd_rdata     (cmd_rdata),
    .cmd_pop       (cmd_pop),
    .cmd_flush     (ctrl_cmd_flush),
    .crx_full      (crx_full),
    .crx_push      (crx_push),
    .crx_wdata     (crx_wdata),
    .scl_f         (scl_f),
    .sda_f         (sda_f),
    .stop_det      (stop_det),
    .bus_free      (bus_free),
    .hs_active     (hs_active),
    .scl_tout      (scl_tout),
    .bus_err       (bus_err),
    .tim_fs        (tim_fs),
    .tim_hs        (tim_hs),
    .scl_oe        (ctrl_scl_oe),
    .sda_oe        (ctrl_sda_oe),
    .cs_en         (ctrl_cs_en),
    .owner         (ctrl_owner),
    .busclr_active (ctrl_busclr),
    .hold          (ctrl_hold),
    .abort_clr     (ctrl_abort_clr),
    .busclr_clr    (ctrl_busclr_clr),
    .ev_cmpl       (ctrl_ev_cmpl),
    .ev_nack       (ctrl_ev_nack),
    .ev_arb        (ctrl_ev_arb),
    .ev_cmderr     (ctrl_ev_cmderr),
    .ev_stall      (ctrl_ev_stall),
    .ev_bcdone     (ctrl_ev_bcdone),
    .cstat_set     (ctrl_cstat_set)
  );

  //---------------------------------------------------------------------------
  // Target
  //---------------------------------------------------------------------------
  i2c_tgt u_tgt (
    .clk         (PCLK),
    .rst_n       (PRESETn),
    .en          (core_en),
    .soft_rst    (swrst),
    .tgt_en      (cfg_tgt_en),
    .scl_f       (scl_f),
    .sda_f       (sda_f),
    .scl_rise    (scl_rise),
    .scl_fall    (scl_fall),
    .start_det   (start_det),
    .stop_det    (stop_det),
    .bus_busy    (bus_busy),
    .scl_tout    (scl_tout),
    .ctrl_owner  (ctrl_owner),
    .addr0       (taddr0),
    .addr0_en    (taddr0_en),
    .addr1       (taddr1),
    .addr1_en    (taddr1_en),
    .gc_en       (gc_en),
    .nostretch   (nostretch),
    .tnack       (tnack),
    .devid_en    (devid_en),
    .devid       (devid),
    .t_hddat     (hs_active ? tim_hs.thd_dat : tim_fs.thd_dat),
    .t_sudat     (hs_active ? tim_hs.tsu_dat : tim_fs.tsu_dat),
    .trx_free1   (~trx_full),
    .trx_free2   (trx_lvl <= (DEPTH6 - 6'd2)),
    .trx_push    (trx_push),
    .trx_wdata   (trx_wdata),
    .ttx_empty   (ttx_empty),
    .ttx_rdata   (ttx_rdata),
    .ttx_pop     (ttx_pop),
    .scl_oe      (tgt_scl_oe),
    .sda_oe      (tgt_sda_oe),
    .ev_tx_req   (tgt_ev_tx_req),
    .ev_taddr    (tgt_ev_taddr),
    .ev_tstop    (tgt_ev_tstop),
    .ev_ttx_done (tgt_ev_ttx_done),
    .ev_gc_swrst (tgt_ev_gc_swrst),
    .ev_trx_ovf  (tgt_ev_trx_ovf),
    .ev_ttx_unf  (tgt_ev_ttx_unf),
    .active      (tgt_active),
    .rw          (tgt_rw),
    .hold        (tgt_hold),
    .tstat       (tstat)
  );

  //---------------------------------------------------------------------------
  // Interrupt events at INTR_STAT bit positions (bit 20 FIFO_ERR and bit 21
  // DMA_DONE are generated in i2c_regs; level bits 0, 1, 8, 9 are 0 here)
  //---------------------------------------------------------------------------
  always_comb begin : p_hw_ev
    hw_ev                 = '0;
    hw_ev[IR_CMPL]        = ctrl_ev_cmpl;
    hw_ev[IR_NACK]        = ctrl_ev_nack;
    hw_ev[IR_ARB_LOST]    = ctrl_ev_arb;
    hw_ev[IR_CMD_ERR]     = ctrl_ev_cmderr;
    hw_ev[IR_CMD_STALL]   = ctrl_ev_stall;
    hw_ev[IR_BUSCLR_DONE] = ctrl_ev_bcdone;
    hw_ev[IR_TX_REQ]      = tgt_ev_tx_req;
    hw_ev[IR_TADDR]       = tgt_ev_taddr;
    hw_ev[IR_TSTOP]       = tgt_ev_tstop;
    hw_ev[IR_TTX_DONE]    = tgt_ev_ttx_done;
    hw_ev[IR_GC_SWRST]    = tgt_ev_gc_swrst;
    hw_ev[IR_HS_DET]      = hs_det;
    hw_ev[IR_TRX_OVF]     = tgt_ev_trx_ovf;
    hw_ev[IR_TTX_UNF]     = tgt_ev_ttx_unf;
    hw_ev[IR_SCL_TOUT]    = scl_tout;
    hw_ev[IR_BUS_ERR]     = bus_err;
  end

  //---------------------------------------------------------------------------
  // DMA channel - FIFO selected by DMA_CTRL.SEL
  //   TX FIFOs (CMD, TTX): avail = free entries, DMAC is flow controller
  //   RX FIFOs (CRX, TRX): avail = entries, optional peripheral flow control
  //---------------------------------------------------------------------------
  always_comb begin : p_dma_mux
    unique case (dma_sel)
      2'd0:    begin dma_avail = DEPTH6 - cmd_lvl; dma_lpos = 6'd0;         end
      2'd1:    begin dma_avail = crx_lvl;          dma_lpos = crx_last_pos; end
      2'd2:    begin dma_avail = DEPTH6 - ttx_lvl; dma_lpos = 6'd0;         end
      default: begin dma_avail = trx_lvl;          dma_lpos = trx_stop_pos; end
    endcase
  end

  i2c_dma_ch u_dma_ch (
    .clk        (PCLK),
    .rst_n      (PRESETn),
    .en         (dma_en),
    .pfc        (dma_pfc_cfg & dma_sel[0]),                   // RX FIFOs only
    .end_en     (dma_end_en_cfg),
    .clr        (dma_clr),
    .blk        ((dma_blk_cfg > DEPTH6) ? DEPTH6 : dma_blk_cfg),
    .avail      (dma_avail),
    .lpos       (dma_lpos),
    .req_o      (dma_req_o),
    .req_type_o (dma_req_type_o),
    .ack_i      (dma_ack_i),
    .ack_type_i (dma_ack_type_i),
    .end_req_i  (dma_end_req_i),
    .end_ack_o  (dma_end_ack_o),
    .done       (dma_done),
    .deny       (dma_deny),
    .end_ev     (dma_end_ev),
    .state      (dma_state)
  );

  //---------------------------------------------------------------------------
  // Output stage (registered, glitch free)
  //---------------------------------------------------------------------------
  always_comb begin : p_out_nxt
    if (!cfg_en) begin
      scl_oe_nxt = ovrd_en & ~scl_ovrd;
      sda_oe_nxt = ovrd_en & ~sda_ovrd;
      cs_en_nxt  = 1'b0;
    end else begin
      scl_oe_nxt = ctrl_scl_oe | (tgt_scl_oe & ~ctrl_owner);
      sda_oe_nxt = ctrl_sda_oe | (tgt_sda_oe & ~ctrl_owner);
      cs_en_nxt  = ctrl_cs_en & ~scl_oe_nxt;
    end
  end

  always_ff @(posedge PCLK or negedge PRESETn) begin : p_out
    if (!PRESETn) begin
      scl_oe_o    <= 1'b0;
      sda_oe_o    <= 1'b0;
      scl_cs_en_o <= 1'b0;
      hs_mode_o   <= 1'b0;
    end else begin
      scl_oe_o    <= scl_oe_nxt;
      sda_oe_o    <= sda_oe_nxt;
      scl_cs_en_o <= cs_en_nxt;
      hs_mode_o   <= hs_active;
    end
  end

  //---------------------------------------------------------------------------
  // Status outputs for SoC logic (registered: 1 PCLK after the internal value)
  //   bus / controller / target activity, SCL stretch waiting for service,
  //   FIFO served by the DMA channel, raw INTR_STAT
  //---------------------------------------------------------------------------
  always_ff @(posedge PCLK or negedge PRESETn) begin : p_status
    if (!PRESETn) begin
      debug_bus_busy_o    <= 1'b0;
      debug_ctrl_active_o <= 1'b0;
      debug_ctrl_hold_o   <= 1'b0;
      debug_tgt_active_o  <= 1'b0;
      debug_tgt_hold_o    <= 1'b0;
      debug_dma_sel_o     <= 2'd0;
      debug_intr_stat_o   <= 22'd0;
    end else begin
      debug_bus_busy_o    <= bus_busy;
      debug_ctrl_active_o <= ctrl_owner | ctrl_busclr;
      debug_ctrl_hold_o   <= ctrl_hold;
      debug_tgt_active_o  <= tgt_active;
      debug_tgt_hold_o    <= tgt_hold;
      debug_dma_sel_o     <= dma_sel;
      debug_intr_stat_o   <= intr_stat;
    end
  end

endmodule
