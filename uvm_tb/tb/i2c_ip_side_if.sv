// -----------------------------------------------------------------------------
// i2c_ip_side_if - sideband / status ports of apb_i2c (spec 3.2, 3.3, 3.5)
//   pad outputs (observed), current-source enable, Hs-mode, interrupt lines and every debug_* port.
//   Passive: only the env side monitor and the tests read it.
// -----------------------------------------------------------------------------
`ifndef I2C_IP_SIDE_IF_SV
`define I2C_IP_SIDE_IF_SV

interface i2c_ip_side_if (input logic clk, input logic rst_n);
  logic        scl_oe, sda_oe, cs_en, hs_mode;
  logic        irq, irq_ctrl, irq_tgt, irq_err;
  logic        d_bus_busy, d_bus_free, d_hs_active, d_scl, d_sda, d_start, d_stop;
  logic [3:0]  d_bit_cnt;
  logic        d_ctrl_active, d_ctrl_hold;
  logic [3:0]  d_ctrl_state;
  logic        d_tgt_active, d_tgt_rw, d_tgt_hold;
  logic [2:0]  d_tgt_state;
  logic [5:0]  d_cmd_lvl, d_crx_lvl, d_ttx_lvl, d_trx_lvl;
  logic [1:0]  d_dma_state, d_dma_sel;
  logic [21:0] d_intr_stat;

  clocking mon_cb @(posedge clk);
    default input #1step;
    input scl_oe, sda_oe, cs_en, hs_mode, irq, irq_ctrl, irq_tgt, irq_err;
    input d_bus_busy, d_bus_free, d_hs_active, d_scl, d_sda, d_start, d_stop, d_bit_cnt;
    input d_ctrl_active, d_ctrl_hold, d_ctrl_state, d_tgt_active, d_tgt_rw, d_tgt_hold, d_tgt_state;
    input d_cmd_lvl, d_crx_lvl, d_ttx_lvl, d_trx_lvl, d_dma_state, d_dma_sel, d_intr_stat;
  endclocking
endinterface : i2c_ip_side_if

`endif
