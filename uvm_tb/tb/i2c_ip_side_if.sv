// -----------------------------------------------------------------------------
// i2c_ip_side_if - sideband / status ports of apb_i2c (spec 3.2, 3.3, 3.5)
//   pad outputs (observed), current-source enable, Hs-mode, interrupt lines and the status ports debug_*.
//   b_* : TB observation of the I2C bus lines (bus_scl / bus_sda = the open-drain wires), independent of the DUT:
//         b_start / b_stop  1-cycle pulse after a START (or Sr) / STOP on the bus (sampled with clk)
//         b_bit_cnt         SCL rising edges since START / since the ACK clock (1..9), 0 after STOP
//   Passive: only the env side monitor and the tests read it.
// -----------------------------------------------------------------------------
`ifndef I2C_IP_SIDE_IF_SV
`define I2C_IP_SIDE_IF_SV

interface i2c_ip_side_if (input logic clk, input logic rst_n);
  logic        scl_oe, sda_oe, cs_en, hs_mode;
  logic        irq, irq_ctrl, irq_tgt, irq_err;
  logic        d_bus_busy, d_ctrl_active, d_ctrl_hold, d_tgt_active, d_tgt_hold;
  logic [1:0]  d_dma_sel;
  logic [21:0] d_intr_stat;

  // ---------------------------------------------------------------- bus observation (TB only)
  logic        bus_scl, bus_sda;
  logic        b_scl_p = 1'b1, b_sda_p = 1'b1;
  logic        b_start = 1'b0, b_stop = 1'b0;
  logic [3:0]  b_bit_cnt = 4'd0;

  always @(posedge clk) begin
    b_start <= (bus_scl === 1'b1) && (b_scl_p === 1'b1) && (b_sda_p === 1'b1) && (bus_sda === 1'b0);
    b_stop  <= (bus_scl === 1'b1) && (b_scl_p === 1'b1) && (b_sda_p === 1'b0) && (bus_sda === 1'b1);
    if ((bus_scl === 1'b1) && (b_scl_p === 1'b1) && (b_sda_p !== bus_sda))  b_bit_cnt <= 4'd0;
    else if ((bus_scl === 1'b1) && (b_scl_p !== 1'b1))                     b_bit_cnt <= (b_bit_cnt == 4'd9) ? 4'd1 : b_bit_cnt + 4'd1;
    b_scl_p <= bus_scl;
    b_sda_p <= bus_sda;
  end

  clocking mon_cb @(posedge clk);
    default input #1step;
    input scl_oe, sda_oe, cs_en, hs_mode, irq, irq_ctrl, irq_tgt, irq_err;
    input d_bus_busy, d_ctrl_active, d_ctrl_hold, d_tgt_active, d_tgt_hold, d_dma_sel, d_intr_stat;
    input b_start, b_stop, b_bit_cnt;
  endclocking
endinterface : i2c_ip_side_if

`endif
