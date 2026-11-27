//-----------------------------------------------------------------------------
// i2c_qch
//   Arm Low Power Interface (LPI) Q-Channel - device side (spec 3.8, 4.12).
//   Controls the gating of PCLK by an external clock / power controller.
//
//   Handshake states, encoded by {QREQn, QACCEPTn, QDENY}:
//     Q_RUN      1 1 0     Q_REQUEST  0 1 0     Q_STOPPED  0 0 0
//     Q_EXIT     1 0 0     Q_DENIED   0 1 1     Q_CONTINUE 1 1 1
//
//   Q_REQUEST is answered in one cycle (after the QREQn synchronizer):
//     accept (QACCEPTn 1 -> 0) when the IP has been quiescent for at least
//     IDLE_CYC consecutive cycles, otherwise deny (QDENY 0 -> 1).
//   Q_EXIT    : QACCEPTn 0 -> 1 once QREQn = 1 is seen.
//   Q_CONTINUE: QDENY    1 -> 0 once QREQn = 1 is seen.
//
//   QREQn is asynchronous: 2-FF synchronizer, reset to QREQN_RST (the level
//   the LPI controller drives while PRESETn is asserted). QACCEPTn resets to
//   QREQN_RST too (Q_RUN or Q_STOPPED at reset), QDENY resets to 0.
//   QACCEPTn and QDENY come straight from flops.
//-----------------------------------------------------------------------------
module i2c_qch #(
  parameter logic QREQN_RST = 1'b1,
  parameter int   IDLE_CYC  = 2          // >= 1
) (
  input  logic clk,
  input  logic rst_n,
  input  logic qreqn_i,       // asynchronous
  input  logic idle,          // IP quiescent this cycle (synchronous)
  output logic qacceptn_o,
  output logic qdeny_o,
  output logic run            // QACCEPTn = 1: Q_RUN / Q_REQUEST / Q_DENIED / Q_CONTINUE
);

  if (IDLE_CYC < 1) begin : g_bad_idle
    $fatal(1, "i2c_qch: IDLE_CYC must be >= 1");
  end

  //---------------------------------------------------------------------------
  // QREQn synchronizer
  //---------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic r_qreqn_s1;
  (* ASYNC_REG = "TRUE" *) logic r_qreqn_s2;

  always_ff @(posedge clk or negedge rst_n) begin : p_sync
    if (!rst_n) begin
      r_qreqn_s1 <= QREQN_RST;
      r_qreqn_s2 <= QREQN_RST;
    end else begin
      r_qreqn_s1 <= qreqn_i;
      r_qreqn_s2 <= r_qreqn_s1;
    end
  end

  //---------------------------------------------------------------------------
  // Quiescence counter: saturates at IDLE_CYC, cleared by any busy cycle.
  //   Guarantees that the flops one or two stages behind the core state
  //   (INTR_STAT -> irq_*, debug_*, pad output flops) have settled before the
  //   clock may be stopped.
  //---------------------------------------------------------------------------
  localparam int CW = $clog2(IDLE_CYC + 1);

  logic [CW-1:0] r_idle_cnt;
  logic          quiet;

  assign quiet = idle & (r_idle_cnt == CW'(IDLE_CYC));

  always_ff @(posedge clk or negedge rst_n) begin : p_idle_cnt
    if (!rst_n)                          r_idle_cnt <= '0;
    else if (!idle)                      r_idle_cnt <= '0;
    else if (r_idle_cnt != CW'(IDLE_CYC)) r_idle_cnt <= r_idle_cnt + 1'b1;
  end

  //---------------------------------------------------------------------------
  // Handshake (flat, mutually exclusive strobes)
  //---------------------------------------------------------------------------
  logic r_qacceptn;
  logic r_qdeny;
  logic in_req;       // Q_REQUEST
  logic do_accept;    // Q_REQUEST -> Q_STOPPED
  logic do_deny;      // Q_REQUEST -> Q_DENIED
  logic do_exit;      // Q_EXIT    -> Q_RUN
  logic do_cont;      // Q_CONTINUE -> Q_RUN

  assign in_req    =  r_qacceptn & ~r_qdeny & ~r_qreqn_s2;
  assign do_accept =  in_req &  quiet;
  assign do_deny   =  in_req & ~quiet;
  assign do_exit   = ~r_qacceptn &  r_qreqn_s2;
  assign do_cont   =  r_qdeny    &  r_qreqn_s2;

  always_ff @(posedge clk or negedge rst_n) begin : p_qacceptn
    if (!rst_n)         r_qacceptn <= QREQN_RST;
    else if (do_accept) r_qacceptn <= 1'b0;
    else if (do_exit)   r_qacceptn <= 1'b1;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_qdeny
    if (!rst_n)       r_qdeny <= 1'b0;
    else if (do_deny) r_qdeny <= 1'b1;
    else if (do_cont) r_qdeny <= 1'b0;
  end

  assign qacceptn_o = r_qacceptn;
  assign qdeny_o    = r_qdeny;
  assign run        = r_qacceptn;

endmodule
