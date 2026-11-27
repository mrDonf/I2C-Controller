// -----------------------------------------------------------------------------
// i2c_ip_seq_pkg - sequence library of the apb_i2c environment
//
//   layer 0  base/     i2c_ip_base_vseq: driver-model API (APB / register model access, checks, event waits,
//                      controller / target / external-controller helpers)
//   layer 1  cfg/      configuration sequences (init, Hs, timeout, FIFO thresholds, external devices)
//            xfer/     traffic sequences (controller transfer / commands, external transfer, TRX collector,
//                      TX_REQ service, DMA start / check)
//            bus/      bit-bang stimuli (arbitration contender, clock stretching, glitches, stuck lines)
//            util/     timing measurement, bus recovery, PRESETn, end-of-test checks
//            lp/       Q-Channel quiescence cycle, background LPI clock controller
//   layer 2  feature/  one self-checking virtual sequence per verification-plan item (reg, intr/FIFO, ctrl,
//                      tgt, bus, dma, qch)
//            rand/     random traffic / random register sequences
//   The tests (i2c_ip_test_pkg) compose layer-1 and layer-2 sequences; the agent-level sequences (APB VIP,
//   I2C VIP API, bit-bang API, DMAC operations) are layer -1 and are only used through layer 0.
// -----------------------------------------------------------------------------
`ifndef I2C_IP_SEQ_PKG_SV
`define I2C_IP_SEQ_PKG_SV

`include "uvm_macros.svh"

package i2c_ip_seq_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;
  import apb_pkg::*;
  import i2c_pkg::*;
  import i2c_dma_pkg::*;
  import i2c_bb_pkg::*;
  import i2c_qch_pkg::*;
  import i2c_ip_ral_pkg::*;
  import i2c_ip_env_pkg::*;

  // layer 0
  `include "base/i2c_ip_base_vseq.svh"
  // layer 1
  `include "cfg/i2c_ip_cfg_seqs.svh"
  `include "xfer/i2c_ip_xfer_seqs.svh"
  `include "bus/i2c_ip_bb_seqs.svh"
  `include "util/i2c_ip_util_seqs.svh"
  `include "lp/i2c_ip_qch_seqs.svh"
  // layer 2
  `include "feature/i2c_ip_reg_vseqs.svh"
  `include "feature/i2c_ip_intr_vseqs.svh"
  `include "feature/i2c_ip_ctrl_vseqs.svh"
  `include "feature/i2c_ip_tgt_vseqs.svh"
  `include "feature/i2c_ip_bus_vseqs.svh"
  `include "feature/i2c_ip_dma_vseqs.svh"
  `include "rand/i2c_ip_rand_vseqs.svh"
  `include "feature/i2c_ip_qch_vseqs.svh"
endpackage : i2c_ip_seq_pkg

`endif
