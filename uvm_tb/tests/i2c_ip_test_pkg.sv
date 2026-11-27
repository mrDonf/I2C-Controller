// -----------------------------------------------------------------------------
// i2c_ip_test_pkg - tests of apb_i2c
//
//   i2c_ip_base_test (i2c_ip_base_test.svh) builds the environment, initialises the DUT in the configure phase,
//   runs the scenario of the test in the main phase and the end-of-test checks in the shutdown phase.
//   Feature-group bases: i2c_reg_base_test, i2c_ctrl_base_test, i2c_tgt_base_test, i2c_bus_base_test,
//   i2c_dma_base_test, i2c_qch_base_test. Every test (one file per test under reg/ intr/ ctrl/ tgt/ bus/ dma/ rand/
//   qch/) is one item of
//   the verification plan (docs/vplan.md) and composes its own layer-1 / layer-2 sequences in run_scenario().
// -----------------------------------------------------------------------------
`ifndef I2C_IP_TEST_PKG_SV
`define I2C_IP_TEST_PKG_SV

`include "uvm_macros.svh"

package i2c_ip_test_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;
  import apb_pkg::*;
  import i2c_pkg::*;
  import i2c_dma_pkg::*;
  import i2c_bb_pkg::*;
  import i2c_qch_pkg::*;
  import i2c_ip_ral_pkg::*;
  import i2c_ip_env_pkg::*;
  import i2c_ip_seq_pkg::*;

  `include "i2c_ip_base_test.svh"

  // registers / APB
  `include "reg/i2c_reg_hw_reset_test.svh"
  `include "reg/i2c_reg_presetn_test.svh"
  `include "reg/i2c_reg_bit_bash_test.svh"
  `include "reg/i2c_reg_strobe_test.svh"
  `include "reg/i2c_reg_lock_test.svh"
  `include "reg/i2c_reg_access_err_test.svh"
  `include "reg/i2c_reg_w1c_test.svh"
  `include "reg/i2c_reg_rand_test.svh"

  // interrupts / FIFOs
  `include "intr/i2c_intr_test.svh"
  `include "intr/i2c_intr_rand_test.svh"
  `include "intr/i2c_fifo_cmd_test.svh"
  `include "intr/i2c_fifo_ttx_test.svh"
  `include "intr/i2c_fifo_rx_test.svh"

  // controller
  `include "ctrl/i2c_ctrl_write_test.svh"
  `include "ctrl/i2c_ctrl_read_test.svh"
  `include "ctrl/i2c_ctrl_write_read_test.svh"
  `include "ctrl/i2c_ctrl_irq_test.svh"
  `include "ctrl/i2c_ctrl_timing_test.svh"
  `include "ctrl/i2c_ctrl_timing_sm_test.svh"
  `include "ctrl/i2c_ctrl_timing_fm_test.svh"
  `include "ctrl/i2c_ctrl_timing_fmp_test.svh"
  `include "ctrl/i2c_ctrl_cmd_stall_test.svh"
  `include "ctrl/i2c_ctrl_crx_full_test.svh"
  `include "ctrl/i2c_ctrl_nack_test.svh"
  `include "ctrl/i2c_ctrl_data_nack_test.svh"
  `include "ctrl/i2c_ctrl_nakok_test.svh"
  `include "ctrl/i2c_ctrl_cmderr_test.svh"
  `include "ctrl/i2c_ctrl_abort_test.svh"
  `include "ctrl/i2c_ctrl_disable_test.svh"
  `include "ctrl/i2c_ctrl_start_byte_test.svh"
  `include "ctrl/i2c_ctrl_gcall_test.svh"
  `include "ctrl/i2c_ctrl_hs_test.svh"
  `include "ctrl/i2c_ctrl_rstart_chain_test.svh"
  `include "ctrl/i2c_ctrl_back2back_test.svh"

  // target
  `include "tgt/i2c_tgt_write_test.svh"
  `include "tgt/i2c_tgt_rstart_test.svh"
  `include "tgt/i2c_tgt_addr_filter_test.svh"
  `include "tgt/i2c_tgt_trx_queue_test.svh"
  `include "tgt/i2c_tgt_read_test.svh"
  `include "tgt/i2c_tgt_tx_req_test.svh"
  `include "tgt/i2c_tgt_serviced_read_test.svh"
  `include "tgt/i2c_tgt_stretch_test.svh"
  `include "tgt/i2c_tgt_nostretch_test.svh"
  `include "tgt/i2c_tgt_tnack_test.svh"
  `include "tgt/i2c_tgt_gcall_test.svh"
  `include "tgt/i2c_tgt_rsvd_addr_test.svh"
  `include "tgt/i2c_tgt_addr10_test.svh"
  `include "tgt/i2c_tgt_devid_test.svh"
  `include "tgt/i2c_tgt_rand_addr_test.svh"
  `include "tgt/i2c_dual_role_test.svh"
  `include "tgt/i2c_tgt_addr_hold_test.svh"
  `include "tgt/i2c_tgt_combined_test.svh"
  `include "tgt/i2c_tgt_devid_corner_test.svh"

  // bus level
  `include "bus/i2c_arb_lost_addr_test.svh"
  `include "bus/i2c_arb_win_test.svh"
  `include "bus/i2c_arb_lost_data_test.svh"
  `include "bus/i2c_clk_sync_test.svh"
  `include "bus/i2c_busclr_test.svh"
  `include "bus/i2c_scl_timeout_test.svh"
  `include "bus/i2c_bus_err_test.svh"
  `include "bus/i2c_filter_test.svh"
  `include "bus/i2c_busmon_test.svh"
  `include "bus/i2c_bus_busy_test.svh"
  `include "bus/i2c_swrst_test.svh"
  `include "bus/i2c_en_test.svh"
  `include "bus/i2c_arb_lost_sr_test.svh"
  `include "bus/i2c_arb_lost_nack_test.svh"
  `include "bus/i2c_arb_lost_stop_test.svh"
  `include "bus/i2c_arb_lost_sp_test.svh"
  `include "bus/i2c_arb_hold_test.svh"
  `include "bus/i2c_bus_err_ttx_test.svh"

  // DMA
  `include "dma/i2c_dma_cmd_test.svh"
  `include "dma/i2c_dma_crx_test.svh"
  `include "dma/i2c_dma_trx_test.svh"
  `include "dma/i2c_dma_ttx_test.svh"
  `include "dma/i2c_dma_blk_test.svh"
  `include "dma/i2c_dma_req_hold_test.svh"
  `include "dma/i2c_dma_deny_test.svh"
  `include "dma/i2c_dma_ack_type_test.svh"
  `include "dma/i2c_dma_abort_test.svh"
  `include "dma/i2c_dma_rand_test.svh"
  `include "dma/i2c_dma_pfc_late_test.svh"
  `include "dma/i2c_dma_end_test.svh"
  `include "dma/i2c_dma_clr_test.svh"

  // random
  `include "rand/i2c_rand_test.svh"
  `include "rand/i2c_rand_ctrl_test.svh"
  `include "rand/i2c_rand_tgt_test.svh"
  `include "rand/i2c_rand_stretch_test.svh"
  `include "rand/i2c_rand_reconfig_test.svh"

  // Q-Channel (low power)
  `include "qch/i2c_qch_base_test.svh"
  `include "qch/i2c_qch_handshake_test.svh"
  `include "qch/i2c_qch_deny_test.svh"
  `include "qch/i2c_qch_apb_wake_test.svh"
  `include "qch/i2c_qch_bus_wake_test.svh"
  `include "qch/i2c_qch_ctrl_test.svh"
  `include "qch/i2c_qch_dma_test.svh"
  `include "qch/i2c_qch_reset_test.svh"
  `include "qch/i2c_qch_rand_test.svh"
endpackage : i2c_ip_test_pkg

`endif
