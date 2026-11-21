// -----------------------------------------------------------------------------
// i2c_ip_env_pkg - verification environment of apb_i2c
//
//   i2c_ip_tb_params      DUT parameters as instantiated by tb_top
//   i2c_ip_env_cfg        configuration of the env and of every agent
//   i2c_ip_side_monitor   sideband / debug port monitor (interrupt events, START detect, controller ownership history)
//   i2c_ip_scoreboard     spec-based reference model: CMD stream -> bus, bus -> CRX, bus -> TRX, TTX -> bus,
//                         Device ID, address decode, APB access rules
//   i2c_ip_coverage       functional coverage of the IP features
//   i2c_ip_vsqr           virtual sequencer
//   i2c_ip_env            APB VIP env + I2C VIP env + DMAC agent + bit-bang agent + RAL + model + coverage
// -----------------------------------------------------------------------------
`ifndef I2C_IP_ENV_PKG_SV
`define I2C_IP_ENV_PKG_SV

`include "uvm_macros.svh"

package i2c_ip_env_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;
  import apb_pkg::*;
  import i2c_pkg::*;
  import i2c_dma_pkg::*;
  import i2c_bb_pkg::*;
  import i2c_ip_ral_pkg::*;

  typedef virtual i2c_ip_side_if side_vif_t;

  // ---------------------------------------------------------------- register offsets (spec 5.1)
  localparam bit [11:0] R_CTRL = 'h000, R_STATUS = 'h004, R_INTR_STAT = 'h008, R_INTR_EN = 'h00C, R_INTR_TEST = 'h010,
                        R_FIFO_CTRL = 'h014, R_FIFO_STAT = 'h018, R_CMD = 'h01C, R_CRXDATA = 'h020, R_TTXDATA = 'h024,
                        R_TRXDATA = 'h028, R_TADDR = 'h02C, R_TCFG = 'h030, R_HSCFG = 'h034, R_CSTAT = 'h038,
                        R_TSTAT = 'h03C, R_TIM_SCL = 'h040, R_TIM_STA = 'h044, R_TIM_STO = 'h048, R_TIM_DAT = 'h04C,
                        R_HS_TIM_SCL = 'h050, R_HS_TIM_STA = 'h054, R_HS_TIM_STO = 'h058, R_HS_TIM_DAT = 'h05C,
                        R_FILTER = 'h060, R_TIMEOUT = 'h064, R_BUSIDLE = 'h068, R_DEVID = 'h06C, R_BUSMON = 'h070,
                        R_DMA_CTRL = 'h074, R_DMA_BLK = 'h078, R_DMA_STAT = 'h07C, R_PARAM = 'h0F8, R_VERSION = 'h0FC;

  // INTR_STAT bits (spec 4.8)
  localparam int I_CMD_THR = 0, I_CRX_THR = 1, I_CMPL = 2, I_NACK = 3, I_ARB_LOST = 4, I_CMD_ERR = 5, I_CMD_STALL = 6,
                 I_BUSCLR_DONE = 7, I_TRX_THR = 8, I_TTX_THR = 9, I_TX_REQ = 10, I_TADDR = 11, I_TSTOP = 12,
                 I_TTX_DONE = 13, I_GC_SWRST = 14, I_HS_DET = 15, I_TRX_OVF = 16, I_TTX_UNF = 17, I_SCL_TOUT = 18,
                 I_BUS_ERR = 19, I_FIFO_ERR = 20, I_DMA_DONE = 21;
  localparam bit [31:0] EVENT_MASK = 32'h003F_FCFC;

  // CSTAT bits (spec 5.2.13)
  localparam int CS_NACK_ADDR = 0, CS_NACK_DATA = 1, CS_ARB_LOST = 2, CS_USER_ABORT = 3, CS_CMD_ERR = 4,
                 CS_SCL_TOUT = 5, CS_BUS_ERR = 6;

  // CMD bits (spec 4.4.1)
  localparam int C_START = 8, C_STOP = 9, C_READ = 10, C_NACK = 11, C_NAKOK = 12;

  // speed modes of the timing table of spec 6.2 (f_PCLK = 100 MHz)
  typedef enum { SPD_SM, SPD_FM, SPD_FMP } i2c_ip_speed_e;

  typedef struct {
    int unsigned scl_low, scl_high, thd_sta, tsu_sta, tsu_sto, tbuf, thd_dat, tsu_dat, spklen;
  } i2c_ip_timing_t;

  function automatic i2c_ip_timing_t i2c_ip_timing(i2c_ip_speed_e s);
    i2c_ip_timing_t t;
    case (s)
      SPD_SM:  t = '{500, 490, 400, 470, 400, 470, 30, 125, 5};
      SPD_FM:  t = '{135, 100,  60,  60,  60, 130, 30,  40, 5};
      default: t = '{ 50,  34,  26,  26,  26,  50, 10,  17, 5};
    endcase
    return t;
  endfunction

  // Hs-mode timing of spec 6.2: SCL_LOW 15, SCL_HIGH 5, THD_STA/TSU_STA/TSU_STO 16, THD_DAT 1, TSU_DAT 9, SPKLEN_HS 1
  localparam int unsigned HS_L = 15, HS_H = 5, HS_STA = 16, HS_HDDAT = 1, HS_SUDAT = 9, HS_SPK = 1;

  function automatic i2c_speed_e vip_speed(i2c_ip_speed_e s);
    case (s)
      SPD_SM:  return I2C_SM;
      SPD_FM:  return I2C_FM;
      default: return I2C_FMP;
    endcase
  endfunction

  `include "i2c_ip_env_cfg.svh"
  `include "i2c_ip_side_monitor.svh"
  `include "i2c_ip_scoreboard.svh"
  `include "i2c_ip_coverage.svh"
  `include "i2c_ip_vsqr.svh"
  `include "i2c_ip_env.svh"
endpackage : i2c_ip_env_pkg

`endif
