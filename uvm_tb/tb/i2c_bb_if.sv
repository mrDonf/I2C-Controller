// -----------------------------------------------------------------------------
// i2c_bb_if - open-drain pull-downs of the bit-bang / fault-injection agent
//   The top joins them to the bus nets of the I2C VIP interface (assign scl = scl_oe ? 1'b0 : 1'bz);
//   the agent reads the bus levels through the VIP interface.
// -----------------------------------------------------------------------------
`ifndef I2C_BB_IF_SV
`define I2C_BB_IF_SV

interface i2c_bb_if (input logic clk, input logic rst_n);
  logic scl_oe = 1'b0;
  logic sda_oe = 1'b0;
endinterface : i2c_bb_if

`endif
