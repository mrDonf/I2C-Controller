//-----------------------------------------------------------------------------
// apb_i2c_pkg
//   Types and constants shared by the apb_i2c sub-modules.
//-----------------------------------------------------------------------------
package apb_i2c_pkg;

  //---------------------------------------------------------------------------
  // Bus timing set (one for F/S-mode, one for Hs-mode; spec 4.11.2, 5.2.15/16).
  // All values are PCLK cycles; 0 behaves as 1 (spec 7.1).
  //---------------------------------------------------------------------------
  typedef struct packed {
    logic [15:0] scl_low;
    logic [15:0] scl_high;
    logic [15:0] thd_sta;
    logic [15:0] tsu_sta;
    logic [15:0] tsu_sto;
    logic [15:0] thd_dat;
    logic [15:0] tsu_dat;
  } i2c_timing_t;

  //---------------------------------------------------------------------------
  // CMD FIFO entry (spec 4.4.1)
  //---------------------------------------------------------------------------
  localparam int CMD_W     = 13;
  localparam int CMD_START = 8;
  localparam int CMD_STOP  = 9;
  localparam int CMD_READ  = 10;
  localparam int CMD_NACK  = 11;
  localparam int CMD_NAKOK = 12;

  //---------------------------------------------------------------------------
  // CSTAT bits (spec 5.2.13)
  //---------------------------------------------------------------------------
  localparam int CS_NACK_ADDR = 0;
  localparam int CS_NACK_DATA = 1;
  localparam int CS_ARB_LOST  = 2;
  localparam int CS_USER_ABRT = 3;
  localparam int CS_CMD_ERR   = 4;
  localparam int CS_SCL_TOUT  = 5;
  localparam int CS_BUS_ERR   = 6;

  //---------------------------------------------------------------------------
  // INTR_STAT bit positions (spec 4.8)
  //---------------------------------------------------------------------------
  localparam int IR_CMD_THR     = 0;
  localparam int IR_CRX_THR     = 1;
  localparam int IR_CMPL        = 2;
  localparam int IR_NACK        = 3;
  localparam int IR_ARB_LOST    = 4;
  localparam int IR_CMD_ERR     = 5;
  localparam int IR_CMD_STALL   = 6;
  localparam int IR_BUSCLR_DONE = 7;
  localparam int IR_TRX_THR     = 8;
  localparam int IR_TTX_THR     = 9;
  localparam int IR_TX_REQ      = 10;
  localparam int IR_TADDR       = 11;
  localparam int IR_TSTOP       = 12;
  localparam int IR_TTX_DONE    = 13;
  localparam int IR_GC_SWRST    = 14;
  localparam int IR_HS_DET      = 15;
  localparam int IR_TRX_OVF     = 16;
  localparam int IR_TTX_UNF     = 17;
  localparam int IR_SCL_TOUT    = 18;
  localparam int IR_BUS_ERR     = 19;
  localparam int IR_FIFO_ERR    = 20;
  localparam int IR_DMA_DONE    = 21;
  localparam int IR_W           = 22;

  // event bits of INTR_STAT (bits 0, 1, 8, 9 are level bits, not stored)
  localparam logic [IR_W-1:0] IR_EVENT_MASK = 22'h3F_FCFC;

  //---------------------------------------------------------------------------
  // Target RX FIFO tags (spec 4.5.2)
  //---------------------------------------------------------------------------
  localparam logic [1:0] TAG_DATA   = 2'b00;
  localparam logic [1:0] TAG_START  = 2'b01;
  localparam logic [1:0] TAG_STOP   = 2'b10;
  localparam logic [1:0] TAG_RSTART = 2'b11;

  //---------------------------------------------------------------------------
  // FSM encodings (controller spec 4.4.11, target spec 4.5.8)
  //---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    C_IDLE    = 4'd0,
    C_WAITBUS = 4'd1,
    C_STA     = 4'd2,
    C_LOW     = 4'd3,
    C_HIGH    = 4'd4,
    C_SR_H    = 4'd5,
    C_SR_B    = 4'd6,
    C_P_H     = 4'd7,
    C_P_W     = 4'd8
  } ctrl_state_t;

  typedef enum logic [2:0] {
    T_IDLE  = 3'd0,
    T_ADDR  = 3'd1,
    T_RX    = 3'd2,
    T_RXACK = 3'd3,
    T_TX    = 3'd4,
    T_TXACK = 3'd5,
    T_HOLD  = 3'd6,
    T_WAIT  = 3'd7
  } tgt_state_t;

  // DMA handshake states (spec 4.9.3; bit 0 is the request itself)
  typedef enum logic [1:0] {
    D_IDLE = 2'b00,
    D_REQ  = 2'b01,
    D_ACK  = 2'b10
  } dma_state_t;

  // DMA request / acknowledge qualifiers (spec 3.4)
  localparam logic [1:0] RT_SINGLE      = 2'b00;
  localparam logic [1:0] RT_LAST_SINGLE = 2'b01;
  localparam logic [1:0] RT_BLOCK       = 2'b10;
  localparam logic [1:0] RT_LAST_BLOCK  = 2'b11;
  localparam logic [1:0] AT_DENY        = 2'b01;
  localparam logic [1:0] AT_LAST_OKAY   = 2'b10;

  //---------------------------------------------------------------------------
  // Helpers
  //---------------------------------------------------------------------------
  // timing value with "0 behaves as 1"
  function automatic logic [15:0] min1(input logic [15:0] v);
    return (v == 16'd0) ? 16'd1 : v;
  endfunction

endpackage
