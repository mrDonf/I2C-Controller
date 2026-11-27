// -----------------------------------------------------------------------------
// i2c_ip_qch_if - Arm LPI Q-Channel of apb_i2c (spec 3.8, 4.12) + clock gate of the DUT PCLK
//
//   qreqn / cg_en      : driven by the LPI clock-controller model (i2c_qch_pkg::i2c_qch_ctrl)
//   qacceptn/qdeny/qactive : DUT outputs
//   gclk               : PCLK of the DUT = clk gated by an ICG model (latch transparent while clk = 0, AND)
//   o_*                : TB observation of the wake sources and of the IP activity (assigned by tb_top), used by the
//                        controller model to classify a wake-up and to check that an accept happens only when idle
// -----------------------------------------------------------------------------
`ifndef I2C_IP_QCH_IF_SV
`define I2C_IP_QCH_IF_SV

interface i2c_ip_qch_if (input logic clk, input logic rst_n);
  logic qreqn;
  logic qacceptn, qdeny, qactive;
  logic cg_en = 1'b1;
  logic cg_lat;
  logic gclk;

  // observation (TB only)
  logic o_psel, o_scl, o_sda, o_end_req, o_dma_req;
  logic o_bus_busy, o_ctrl_active, o_tgt_active;

  // integrated clock gate: enable latched while clk is low -> glitch-free gated clock
  always_latch if (!clk) cg_lat = cg_en;
  assign gclk = clk & cg_lat;

  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    input  qacceptn, qdeny, qactive;
    input  o_psel, o_scl, o_sda, o_end_req, o_dma_req, o_bus_busy, o_ctrl_active, o_tgt_active;
    output qreqn, cg_en;
  endclocking
endinterface : i2c_ip_qch_if

`endif
