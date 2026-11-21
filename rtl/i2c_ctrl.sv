//-----------------------------------------------------------------------------
// i2c_ctrl (spec 4.4)
//   I2C controller engine.
//   - Executes commands from the CMD FIFO (1 command = 1 byte on the bus)
//   - SCL generation with clock synchronization / stretching tolerance
//   - START / Repeated START / STOP generation
//   - Arbitration (F/S only), Hs-mode controller code, START byte
//   - NACK / command error / user abort handling, bus clear
//
// Structure
//   1. decode     : state one-hots, byte status, timing set selection
//   2. timers     : phase counter compares (flat comparisons, no arithmetic)
//   3. decisions  : one strobe per event, written as flat AND terms; the
//                   strobes of one register are mutually exclusive
//   4. registers  : one always block per group of related registers
//   5. outputs    : FIFO / CSR handshakes, events, bus drive
//
// Phase counter
//   The phase count is 1 in the first cycle a new bus level is observed on
//   scl_f / sda_f; an action decided when the count equals N reaches the pad
//   N+1 cycles (+LAT) after the observed edge (one cycle for this module's
//   register and one for the output register in the top level).
//   r_cnt holds (phase count + 1), saturating at 2^16 (= count 0xFFFF), so
//   every threshold test is a direct comparison with a register value:
//       count >= X - 1   <=>  r_cnt >= X        count >= X  <=>  r_cnt > X
//   r_cnt == 2 means "count 0": the new level has not been observed yet.
//   All phases are mutually exclusive, so one counter serves:
//     C_STA / C_SR_B : SDA observed low   (THD_STA)
//     C_LOW          : SCL observed low   (THD_DAT, SCL_LOW)
//     C_HIGH         : SCL observed high  (SCL_HIGH)
//     C_SR_H / C_P_H : SCL observed high  (TSU_STA / TSU_STO)
//     C_P_W          : SDA released       (bus clear end / STOP watchdog)
//   Timing inputs of 0 behave as 1 (the comparisons give the same result).
//-----------------------------------------------------------------------------
module i2c_ctrl
  import apb_i2c_pkg::*;
(
  input  logic             clk,
  input  logic             rst_n,
  // configuration / control
  input  logic             en,
  input  logic             ctrl_en,
  input  logic             hs_en,
  input  logic             sb_en,
  input  logic [2:0]       mcode,
  input  logic             abort_req,
  input  logic             busclr_req,
  input  i2c_timing_t      tim_fs,
  input  i2c_timing_t      tim_hs,
  // command FIFO
  input  logic             cmd_empty,
  input  logic [CMD_W-1:0] cmd_rdata,
  output logic             cmd_pop,
  output logic             cmd_flush,
  // controller RX FIFO
  input  logic             crx_full,
  output logic             crx_push,
  output logic [8:0]       crx_wdata,      // {LAST, data}
  // bus
  input  logic             scl_f,
  input  logic             sda_f,
  input  logic             stop_det,
  input  logic             bus_free,
  input  logic             hs_active,
  input  logic             scl_tout,
  input  logic             bus_err,
  // bus drive
  output logic             scl_oe,
  output logic             sda_oe,
  output logic             cs_en,
  // status
  output logic             owner,
  output logic             busclr_active,
  output logic             hold,
  output ctrl_state_t      state,
  // handshakes to CSR
  output logic             abort_clr,
  output logic             busclr_clr,
  // events
  output logic             ev_cmpl,
  output logic             ev_nack,
  output logic             ev_arb,
  output logic             ev_cmderr,
  output logic             ev_stall,
  output logic             ev_bcdone,
  output logic [6:0]       cstat_set
);

  // kind of the byte in progress
  typedef enum logic [2:0] {
    K_ADDR = 3'd0,     // address byte
    K_WR   = 3'd1,     // data byte, controller-transmitter
    K_RD   = 3'd2,     // data byte, controller-receiver
    K_MC   = 3'd3,     // Hs controller code
    K_SB   = 3'd4,     // START byte
    K_BC   = 3'd5      // bus clear clocks
  } byte_kind_t;

  // action performed at the end of the current SCL-low phase
  typedef enum logic [1:0] {
    N_WAIT = 2'd0,     // wait for a command (fetch)
    N_BIT  = 2'd1,     // next bit
    N_SR   = 2'd2,     // repeated START
    N_STOP = 2'd3      // STOP
  } low_action_t;

  localparam logic [16:0] CNT_0   = 17'd2;          // phase count 0
  localparam logic [16:0] CNT_1   = 17'd3;          // phase count 1
  localparam logic [16:0] CNT_MAX = 17'h1_0000;     // phase count 0xFFFF

  //---------------------------------------------------------------------------
  // Registers
  //---------------------------------------------------------------------------
  ctrl_state_t r_state,      state_nxt;
  logic [16:0] r_cnt,        cnt_nxt;        // phase count + 1
  byte_kind_t  r_kind,       kind_nxt;
  logic [3:0]  r_bit_idx,    bit_idx_nxt;    // 0..7 data bits, 8 = ACK bit
  logic [7:0]  r_byte,       byte_nxt;       // address / tx data / rx data
  logic        r_dir_rd,     dir_rd_nxt;     // R/W bit of the last address
  logic        r_cmd_stop,   cmd_stop_nxt;   // command has STOP
  logic        r_cmd_nakok,  cmd_nakok_nxt;  // command has NAKOK
  logic        r_rd_nack,    rd_nack_nxt;    // read byte is (to be) NACKed
  low_action_t r_action,     action_nxt;
  logic        r_sda_set,    sda_set_nxt;    // SDA updated for the low action
  logic        r_sda_oe,     sda_oe_nxt;
  logic        r_sda_smp;                    // SDA sampled at SCL rise
  logic        r_abort_pend, abort_pend_nxt; // user abort accepted, in progress
  logic        r_dummy,      dummy_nxt;      // dummy read (abort)
  logic        r_stall,      stall_nxt;      // CMD_STALL already reported
  logic [17:0] r_wd_fs;                      // STOP watchdog limits (F/S, Hs)
  logic [17:0] r_wd_hs;

  //===========================================================================
  // 1. Decode
  //===========================================================================
  logic st_idle, st_wait, st_sta, st_low, st_high, st_sr_h, st_sr_b, st_p_h, st_p_w;
  logic st_valid;

  assign st_idle  = (r_state == C_IDLE);
  assign st_wait  = (r_state == C_WAITBUS);
  assign st_sta   = (r_state == C_STA);
  assign st_low   = (r_state == C_LOW);
  assign st_high  = (r_state == C_HIGH);
  assign st_sr_h  = (r_state == C_SR_H);
  assign st_sr_b  = (r_state == C_SR_B);
  assign st_p_h   = (r_state == C_P_H);
  assign st_p_w   = (r_state == C_P_W);
  assign st_valid = st_idle | st_wait | st_sta | st_low | st_high |
                    st_sr_h | st_sr_b | st_p_h | st_p_w;

  logic       busclr;           // bus clear in progress
  logic       own_bus;          // controller owns the bus
  logic       kind_rd;
  logic       tx_kind;          // byte is driven by this controller
  logic       ack_bit;          // current bit is the ACK bit
  logic       drives_bit;       // current bit is driven by this controller
  logic [7:0] tx_byte;
  logic       bit_val;          // value of the current bit (1 = release SDA)
  logic       rd_ack_pending;   // target is transmitting (last read ACKed)
  logic       rd_nack_no_stop;  // last read NACKed without STOP (Sr must follow)

  assign busclr     = (r_kind == K_BC) & ~st_idle;
  assign own_bus    = ~st_idle & ~st_wait & ~busclr;
  assign kind_rd    = (r_kind == K_RD);
  assign tx_kind    = ~kind_rd & (r_kind != K_BC);
  assign ack_bit    = r_bit_idx[3];
  assign drives_bit = ack_bit ? kind_rd : tx_kind;

  always_comb begin : p_tx_byte
    unique case (r_kind)
      K_MC:    tx_byte = {5'b00001, mcode};
      K_SB:    tx_byte = 8'h01;
      default: tx_byte = r_byte;
    endcase
  end

  always_comb begin : p_bit_val
    if (ack_bit)      bit_val = kind_rd ? r_rd_nack : 1'b1;
    else if (tx_kind) bit_val = tx_byte[~r_bit_idx[2:0]];
    else              bit_val = 1'b1;
  end

  // valid in C_LOW / N_WAIT, after the byte of r_kind has completed
  assign rd_ack_pending  = (kind_rd & ~r_rd_nack) |
                           ((r_kind == K_ADDR) & r_dir_rd & ~r_sda_smp);
  assign rd_nack_no_stop = kind_rd & r_rd_nack & ~r_cmd_stop;

  // timing set: F/S for bus clear and for the low phase after the controller
  // code, Hs while the bus is in Hs-mode
  logic        tsel_hs;
  i2c_timing_t tim;

  assign tsel_hs = hs_active & ~busclr & ~((r_kind == K_MC) & st_low);
  assign tim     = tsel_hs ? tim_hs : tim_fs;

  //===========================================================================
  // 2. Timers
  //===========================================================================
  logic [16:0] cnt_inc;
  logic [16:0] hd_load;          // count THD_DAT (0 as 1), restart point of SCL_LOW
  logic        cnt_is0;          // new level not observed yet
  logic        scl_rose;         // C_HIGH: first cycle SCL is observed high
  logic        smp_now;          // SDA sample of the current bit
  logic        hd_reached;       // C_LOW : count >= THD_DAT
  logic        low_done;         // C_LOW : count >= SCL_LOW  - 1
  logic        high_done;        // C_HIGH: count >= SCL_HIGH - 1
  logic        hdsta_done;       // C_STA / C_SR_B: count >= THD_STA
  logic        susta_done;       // C_SR_H: count >= TSU_STA
  logic        susto_done;       // C_P_H : count >= TSU_STO
  logic        bc_end;           // C_P_W, bus clear: count >= SCL_HIGH
  logic        wd_expired;       // C_P_W, STOP: count >= SCL_LOW + SCL_HIGH + TSU_STO

  assign cnt_inc    = r_cnt[16] ? r_cnt : (r_cnt + 17'd1);
  assign hd_load    = ({1'b0, min1(tim.thd_dat)} == 17'hFFFF) ? CNT_MAX
                                                              : ({1'b0, min1(tim.thd_dat)} + 17'd2);
  assign cnt_is0    = (r_cnt == CNT_0);
  assign scl_rose   = scl_f & cnt_is0;
  assign smp_now    = scl_rose ? sda_f : r_sda_smp;

  assign hd_reached = ~scl_f & (r_cnt >  {1'b0, tim.thd_dat});
  assign low_done   = ~scl_f & (r_cnt >= {1'b0, tim.scl_low});
  assign high_done  =           r_cnt >= {1'b0, tim.scl_high};
  assign hdsta_done =           r_cnt >  {1'b0, tim.thd_sta};
  assign susta_done =           r_cnt >  {1'b0, tim.tsu_sta};
  assign susto_done =           r_cnt >  {1'b0, tim.tsu_sto};
  assign bc_end     =           r_cnt >  {1'b0, tim.scl_high};
  assign wd_expired = {1'b0, r_cnt} > (hs_active ? r_wd_hs : r_wd_fs);

  // STOP watchdog limit per timing set (static while CTRL.EN = 1)
  always_ff @(posedge clk or negedge rst_n) begin : p_wd_limit
    if (!rst_n) begin
      r_wd_fs <= 18'd0;
      r_wd_hs <= 18'd0;
    end else begin
      r_wd_fs <= 18'(min1(tim_fs.scl_low)) + 18'(min1(tim_fs.scl_high)) + 18'(min1(tim_fs.tsu_sto));
      r_wd_hs <= 18'(min1(tim_hs.scl_low)) + 18'(min1(tim_hs.scl_high)) + 18'(min1(tim_hs.tsu_sto));
    end
  end

  //===========================================================================
  // 3. Decisions (spec 7.2 priority: EN, SCL_TOUT, BUS_ERR, then the FSM)
  //===========================================================================
  // global
  logic g_tout, g_berr, run;
  assign g_tout = en & scl_tout & (own_bus | busclr);
  assign g_berr = en & ~g_tout & bus_err & own_bus;
  assign run    = en & ~g_tout & ~g_berr;

  // user abort / controller disable while owning the bus; bus clear when busy
  logic abort_acc, busclr_rej;
  assign abort_acc  = run & own_bus & (abort_req | ~ctrl_en) & ~r_abort_pend;
  assign busclr_rej = run & busclr_req & ~st_idle & ~busclr;

  // C_IDLE
  logic i_bc_go, i_bc_rej, i_abort, i_cmd, i_cmd_bad, i_err, i_go;
  assign i_bc_go   = run & st_idle &  busclr_req & ctrl_en;
  assign i_bc_rej  = run & st_idle &  busclr_req & ~ctrl_en;
  assign i_abort   = run & st_idle & ~busclr_req & abort_req;
  assign i_cmd     = run & st_idle & ~busclr_req & ~abort_req & ctrl_en & ~cmd_empty;
  assign i_cmd_bad = ~cmd_rdata[CMD_START] | cmd_rdata[CMD_READ];        // E1, E5
  assign i_err     = i_cmd &  i_cmd_bad;
  assign i_go      = i_cmd & ~i_cmd_bad;

  // C_WAITBUS
  logic w_abort, w_go;
  assign w_abort = run & st_wait & (abort_req | ~ctrl_en);
  assign w_go    = run & st_wait & ~abort_req & ctrl_en & bus_free;

  // C_STA / C_SR_B: START / Sr hold. SCL pulled low by another device while
  // SDA is low = clock synchronization (both sent S/Sr): the byte starts now.
  logic sta_count, sta_end, sta_sync, sta_arb;
  logic srb_count, srb_end, srb_sync, srb_arb;
  logic begin_byte, begin_sync, begin_first;
  assign sta_count   = run & st_sta  &  scl_f & ~sda_f;
  assign sta_end     = sta_count & hdsta_done;
  assign sta_sync    = run & st_sta  & ~scl_f & ~sda_f;
  assign sta_arb     = run & st_sta  & ~scl_f &  sda_f;
  assign srb_count   = run & st_sr_b &  scl_f & ~sda_f;
  assign srb_end     = srb_count & hdsta_done;
  assign srb_sync    = run & st_sr_b & ~scl_f & ~sda_f;
  assign srb_arb     = run & st_sr_b & ~scl_f &  sda_f & ~hs_active;
  assign begin_sync  = sta_sync | srb_sync;
  assign begin_byte  = sta_end | srb_end | begin_sync;
  assign begin_first = st_sta;                       // byte after a START from idle

  // C_LOW
  logic low_wait, low_setsda, low_end;
  assign low_wait   = run & st_low & (r_action == N_WAIT);
  assign low_setsda = run & st_low & (r_action != N_WAIT) & ~r_sda_set & hd_reached;
  assign low_end    = run & st_low & (r_action != N_WAIT) &  r_sda_set & low_done;

  // C_HIGH: arbitration = SDA released by this controller but low on the bus
  logic hi_lost, hi_rose, hi_arb, hi_done, hi_sync, bit_done, byte_done;
  assign hi_lost   = drives_bit & bit_val & ~sda_f & ~hs_active;
  assign hi_rose   = run & st_high & scl_rose;
  assign hi_arb    = run & st_high & scl_f &  hi_lost;
  assign hi_done   = run & st_high & scl_f & ~hi_lost & high_done;
  assign hi_sync   = run & st_high & ~scl_f & ~cnt_is0;     // SCL pulled low early
  assign bit_done  = hi_done | hi_sync;
  assign byte_done = bit_done & ack_bit;

  // C_SR_H / C_P_H / C_P_W (Hs-mode and bus clear: no arbitration, an early
  // SCL low restarts the setup count)
  logic srh_lost, srh_arb, srh_end, srh_restart;
  logic ph_end, ph_arb, ph_restart;
  logic pw_bc_end, pw_cmpl, pw_arb;
  assign srh_lost    = scl_f ? ~sda_f : ~cnt_is0;
  assign srh_arb     = run & st_sr_h & srh_lost & ~hs_active;
  assign srh_end     = run & st_sr_h & scl_f & ~(~sda_f & ~hs_active) & susta_done;
  assign srh_restart = run & st_sr_h & ~scl_f & ~cnt_is0 & hs_active;
  assign ph_end      = run & st_p_h  &  scl_f & susto_done;
  assign ph_arb      = run & st_p_h  & ~scl_f & ~cnt_is0 & ~busclr & ~hs_active;
  assign ph_restart  = run & st_p_h  & ~scl_f & ~cnt_is0 & (busclr | hs_active);
  assign pw_bc_end   = run & st_p_w  &  busclr & (stop_det | bc_end);
  assign pw_cmpl     = run & st_p_w  & ~busclr & stop_det;
  assign pw_arb      = run & st_p_w  & ~busclr & ~stop_det &
                       ((~scl_f & ~hs_active) | (~sda_f & wd_expired));

  // end of a byte (after the ACK bit)
  logic bd_tx, bd_rd, bd_nack, bd_tx_end, bd_dummy;
  logic bd_to_sr, bd_to_stop, bd_to_wait;
  assign bd_tx      = byte_done & ((r_kind == K_ADDR) | (r_kind == K_WR));
  assign bd_rd      = byte_done & kind_rd;
  assign bd_nack    = bd_tx & smp_now & ~r_cmd_nakok;
  assign bd_tx_end  = bd_tx & ~bd_nack & (r_abort_pend | ~ctrl_en | r_cmd_stop);
  assign bd_dummy   = bd_tx_end & (r_kind == K_ADDR) & r_dir_rd & ~smp_now;
  assign bd_to_sr   = byte_done & ((r_kind == K_MC) | (r_kind == K_SB));
  assign bd_to_stop = bd_nack | (bd_tx_end & ~bd_dummy) |
                      (bd_rd & r_rd_nack & r_cmd_stop) |
                      (byte_done & (r_kind == K_BC));         // 9 clocks done
  assign bd_to_wait = (bd_tx & ~bd_nack & ~bd_tx_end) |
                      (bd_rd & ~(r_rd_nack & r_cmd_stop));

  // fetch of the next command (SCL low, waiting for the next action)
  logic f_abort, f_cmd, f_stall, f_bad, f_err, f_pop, f_end, f_dummy;
  assign f_abort = low_wait & (r_abort_pend | ~ctrl_en);
  assign f_cmd   = low_wait & ~(r_abort_pend | ~ctrl_en) & ~cmd_empty;
  assign f_stall = low_wait & ~(r_abort_pend | ~ctrl_en) &  cmd_empty & ~r_stall;
  assign f_bad   = (cmd_rdata[CMD_START]  &  cmd_rdata[CMD_READ])            |  // E5
                   (cmd_rdata[CMD_START]  &  rd_ack_pending)                 |  // E3
                   (~cmd_rdata[CMD_START] &  rd_nack_no_stop)                |  // E4
                   (~cmd_rdata[CMD_START] & (cmd_rdata[CMD_READ] != r_dir_rd)); // E2
  assign f_err   = f_cmd & f_bad;
  // READ with CRX full: keep SCL low until CRX has room
  assign f_pop   = f_cmd & ~f_bad &
                   (cmd_rdata[CMD_START] | ~cmd_rdata[CMD_READ] | ~crx_full);
  assign f_end   = f_abort | f_err;
  assign f_dummy = f_end & rd_ack_pending;

  // dummy read with NACK + STOP (abort / error while the target transmits)
  logic do_dummy;
  assign do_dummy = bd_dummy | f_dummy;

  // arbitration lost: release the bus immediately
  logic do_arb, go_idle;
  assign do_arb  = sta_arb | srb_arb | hi_arb | srh_arb | ph_arb | pw_arb;
  assign go_idle = ~en | g_tout | g_berr | do_arb | (run & ~st_valid);

  //===========================================================================
  // 4. Registers
  //===========================================================================
  //---------------------------------------------------------------------------
  // FSM state
  //---------------------------------------------------------------------------
  always_comb begin : p_state_nxt
    state_nxt = r_state;
    if (go_idle) begin
      state_nxt = C_IDLE;
    end else begin
      unique case (r_state)
        C_IDLE:
          if (i_bc_go)        state_nxt = C_LOW;           // bus clear
          else if (i_go)      state_nxt = C_WAITBUS;
        C_WAITBUS:
          if (w_abort)        state_nxt = C_IDLE;
          else if (w_go)      state_nxt = C_STA;
        C_STA, C_SR_B:
          if (begin_byte)     state_nxt = C_LOW;
        C_LOW:
          if (low_end) begin
            unique case (r_action)
              N_BIT:          state_nxt = C_HIGH;
              N_SR:           state_nxt = C_SR_H;
              default:        state_nxt = C_P_H;           // N_STOP
            endcase
          end
        C_HIGH:
          if (bit_done)       state_nxt = C_LOW;
        C_SR_H:
          if (srh_end)        state_nxt = C_SR_B;
        C_P_H:
          if (ph_end)         state_nxt = C_P_W;
        C_P_W:
          if (pw_bc_end || pw_cmpl) state_nxt = C_IDLE;
        default:              state_nxt = C_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_state
    if (!rst_n) r_state <= C_IDLE;
    else        r_state <= state_nxt;
  end

  //---------------------------------------------------------------------------
  // Phase counter
  //---------------------------------------------------------------------------
  always_comb begin : p_cnt_nxt
    cnt_nxt = r_cnt;
    if (run) begin
      unique case (r_state)
        C_IDLE:
          if (i_bc_go)                 cnt_nxt = CNT_0;
        C_WAITBUS:
          if (w_go)                    cnt_nxt = CNT_0;
        C_STA:                                              // SDA-low count
          if (sta_sync)                cnt_nxt = CNT_1;
          else if (sta_count)          cnt_nxt = sta_end ? CNT_0 : cnt_inc;
          else if (scl_f)              cnt_nxt = CNT_0;     // SDA not low yet
        C_LOW:                                              // SCL-low count
          if (low_setsda)              cnt_nxt = hd_load;   // SDA changed: restart from THD_DAT
          else if (low_end)            cnt_nxt = CNT_0;
          else if (scl_f)              cnt_nxt = CNT_0;     // SCL not low yet
          else                         cnt_nxt = cnt_inc;
        C_HIGH:                                             // SCL-high count
          if (hi_done)                 cnt_nxt = CNT_0;
          else if (scl_f)              cnt_nxt = cnt_inc;
          else if (hi_sync)            cnt_nxt = CNT_1;     // clock synchronization
        C_SR_H:
          if (srh_end)                 cnt_nxt = CNT_0;
          else if (scl_f)              cnt_nxt = cnt_inc;
          else if (srh_restart)        cnt_nxt = CNT_0;
        C_SR_B:
          if (srb_sync)                cnt_nxt = CNT_1;
          else if (srb_count)          cnt_nxt = srb_end ? CNT_0 : cnt_inc;
        C_P_H:
          if (ph_end)                  cnt_nxt = CNT_0;
          else if (scl_f)              cnt_nxt = cnt_inc;
          else if (ph_restart)         cnt_nxt = CNT_0;
        C_P_W:
          if (busclr)                  cnt_nxt = cnt_inc;
          else if (stop_det)           cnt_nxt = CNT_0;
          else if (scl_f || hs_active) cnt_nxt = cnt_inc;
        default: ;
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_cnt
    if (!rst_n) r_cnt <= CNT_0;
    else        r_cnt <= cnt_nxt;
  end

  //---------------------------------------------------------------------------
  // Byte engine: kind, bit index, shift register, direction
  //---------------------------------------------------------------------------
  logic begin_mc, begin_sb, begin_addr, load_cmd, rx_shift;
  assign begin_mc   = begin_byte & begin_first & hs_en & ~hs_active;
  assign begin_sb   = begin_byte & begin_first & ~(hs_en & ~hs_active) & sb_en;
  assign begin_addr = begin_byte & ~begin_mc & ~begin_sb;
  assign load_cmd   = i_go | f_pop;
  assign rx_shift   = bit_done & ~ack_bit & kind_rd;

  always_comb begin : p_kind_nxt
    unique case (1'b1)
      i_bc_go:                         kind_nxt = K_BC;
      i_go, begin_addr:                kind_nxt = K_ADDR;
      begin_mc:                        kind_nxt = K_MC;
      begin_sb:                        kind_nxt = K_SB;
      do_dummy:                        kind_nxt = K_RD;
      f_pop & ~cmd_rdata[CMD_START]:   kind_nxt = cmd_rdata[CMD_READ] ? K_RD : K_WR;
      default:                         kind_nxt = r_kind;
    endcase
  end

  always_comb begin : p_bit_idx_nxt
    if (i_bc_go | begin_byte | byte_done | f_pop | do_dummy) bit_idx_nxt = 4'd0;
    else if (bit_done)                                       bit_idx_nxt = r_bit_idx + 4'd1;
    else                                                     bit_idx_nxt = r_bit_idx;
  end

  always_comb begin : p_byte_nxt
    if (load_cmd)      byte_nxt = cmd_rdata[7:0];
    else if (rx_shift) byte_nxt = {r_byte[6:0], smp_now};
    else               byte_nxt = r_byte;
  end

  assign dir_rd_nxt = begin_addr ? r_byte[0] : r_dir_rd;

  always_ff @(posedge clk or negedge rst_n) begin : p_byte_engine
    if (!rst_n) begin
      r_kind    <= K_ADDR;
      r_bit_idx <= 4'd0;
      r_byte    <= 8'd0;
      r_dir_rd  <= 1'b0;
    end else begin
      r_kind    <= kind_nxt;
      r_bit_idx <= bit_idx_nxt;
      r_byte    <= byte_nxt;
      r_dir_rd  <= dir_rd_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Command attributes: STOP / NAKOK of the command, NACK of the read byte
  //---------------------------------------------------------------------------
  logic rd_last_bit;      // last data bit of a read: decide ACK / NACK
  assign rd_last_bit = rx_shift & (r_bit_idx == 4'd7);

  assign cmd_stop_nxt  = do_dummy ? 1'b1 : load_cmd ? cmd_rdata[CMD_STOP]  : r_cmd_stop;
  assign cmd_nakok_nxt =                   load_cmd ? cmd_rdata[CMD_NAKOK] : r_cmd_nakok;

  always_comb begin : p_rd_nack_nxt
    if (do_dummy)         rd_nack_nxt = 1'b1;
    else if (f_pop)       rd_nack_nxt = cmd_rdata[CMD_NACK] | cmd_rdata[CMD_STOP];
    else if (rd_last_bit) rd_nack_nxt = r_rd_nack | r_abort_pend | ~ctrl_en;  // abort forces NACK
    else                  rd_nack_nxt = r_rd_nack;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_cmd_attr
    if (!rst_n) begin
      r_cmd_stop  <= 1'b0;
      r_cmd_nakok <= 1'b0;
      r_rd_nack   <= 1'b0;
    end else begin
      r_cmd_stop  <= cmd_stop_nxt;
      r_cmd_nakok <= cmd_nakok_nxt;
      r_rd_nack   <= rd_nack_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Low-phase action and SDA drive
  //---------------------------------------------------------------------------
  logic a_wait, a_bit, a_sr, a_stop;
  assign a_wait = go_idle | bd_to_wait;
  assign a_bit  = i_bc_go | begin_byte | (bit_done & ~ack_bit) | do_dummy |
                  (f_pop & ~cmd_rdata[CMD_START]);
  assign a_sr   = bd_to_sr | (f_pop & cmd_rdata[CMD_START]);
  assign a_stop = bd_to_stop | (f_end & ~rd_ack_pending);

  always_comb begin : p_action_nxt
    unique case (1'b1)
      a_wait:  action_nxt = N_WAIT;
      a_bit:   action_nxt = N_BIT;
      a_sr:    action_nxt = N_SR;
      a_stop:  action_nxt = N_STOP;
      default: action_nxt = r_action;
    endcase
  end

  assign sda_set_nxt = (i_bc_go | begin_byte | bit_done | go_idle) ? 1'b0 :
                       low_setsda                                  ? 1'b1 : r_sda_set;

  // SDA level of the low action: data bit, released for Sr, pulled for STOP
  logic sda_rel, sda_pull, sda_act;
  assign sda_rel  = i_bc_go | (low_wait & hd_reached) | ph_end | go_idle;
  assign sda_pull = w_go | srh_end;
  assign sda_act  = (r_action == N_BIT) ? ~bit_val : (r_action == N_STOP);

  always_comb begin : p_sda_oe_nxt
    unique case (1'b1)
      sda_rel:    sda_oe_nxt = 1'b0;
      sda_pull:   sda_oe_nxt = 1'b1;
      low_setsda: sda_oe_nxt = sda_act;
      default:    sda_oe_nxt = r_sda_oe;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_sda
    if (!rst_n) begin
      r_action  <= N_WAIT;
      r_sda_set <= 1'b0;
      r_sda_oe  <= 1'b0;
      r_sda_smp <= 1'b1;
    end else begin
      r_action  <= action_nxt;
      r_sda_set <= sda_set_nxt;
      r_sda_oe  <= sda_oe_nxt;
      if (hi_rose) r_sda_smp <= sda_f;
    end
  end

  //---------------------------------------------------------------------------
  // Transfer flags: abort in progress, dummy read, CMD_STALL reported
  //---------------------------------------------------------------------------
  assign abort_pend_nxt = (go_idle | pw_cmpl)                       ? 1'b0 :
                          abort_acc                                 ? 1'b1 : r_abort_pend;
  assign dummy_nxt      = (go_idle | bd_rd | (f_pop & ~cmd_rdata[CMD_START])) ? 1'b0 :
                          do_dummy                                  ? 1'b1 : r_dummy;
  assign stall_nxt      = (go_idle | pw_cmpl | f_cmd)               ? 1'b0 :
                          f_stall                                   ? 1'b1 : r_stall;

  always_ff @(posedge clk or negedge rst_n) begin : p_flags
    if (!rst_n) begin
      r_abort_pend <= 1'b0;
      r_dummy      <= 1'b0;
      r_stall      <= 1'b0;
    end else begin
      r_abort_pend <= abort_pend_nxt;
      r_dummy      <= dummy_nxt;
      r_stall      <= stall_nxt;
    end
  end

  //===========================================================================
  // 5. Outputs
  //===========================================================================
  // FIFOs
  assign cmd_pop   = load_cmd;
  assign cmd_flush = (g_tout & own_bus) | g_berr | abort_acc | i_abort | i_err |
                     w_abort | bd_nack | f_err | do_arb;
  assign crx_push  = hi_rose & kind_rd & (r_bit_idx == 4'd7) & ~r_dummy;
  assign crx_wdata = {r_rd_nack, r_byte[6:0], sda_f};   // LAST = final byte of a read

  // CSR handshakes (ABORT / BUSCLR self-clear)
  assign abort_clr  = (~en & abort_req) | i_abort | (w_abort & abort_req) |
                      ((g_tout | g_berr | pw_cmpl | do_arb) & r_abort_pend);
  assign busclr_clr = (~en & busclr_req) | (g_tout & ~own_bus) | busclr_rej |
                      i_bc_rej | pw_bc_end;

  // events
  assign ev_cmpl   = pw_cmpl;
  assign ev_nack   = bd_nack;
  assign ev_arb    = do_arb;
  assign ev_cmderr = i_err | f_err;
  assign ev_stall  = f_stall;
  assign ev_bcdone = (g_tout & ~own_bus) | pw_bc_end;

  always_comb begin : p_cstat_set
    cstat_set               = 7'd0;
    cstat_set[CS_NACK_ADDR] = bd_nack & (r_kind == K_ADDR);
    cstat_set[CS_NACK_DATA] = bd_nack & (r_kind == K_WR);
    cstat_set[CS_ARB_LOST]  = do_arb;
    cstat_set[CS_USER_ABRT] = abort_acc | i_abort | w_abort;
    cstat_set[CS_CMD_ERR]   = i_err | f_err;
    cstat_set[CS_SCL_TOUT]  = g_tout & own_bus;
    cstat_set[CS_BUS_ERR]   = g_berr;
  end

  // bus drive and status
  assign scl_oe        = st_low;                // SCL is pulled in every low phase
  assign sda_oe        = r_sda_oe;
  assign owner         = own_bus;
  assign busclr_active = busclr;
  assign hold          = st_low & (r_action == N_WAIT) & own_bus;
  assign state         = r_state;

  // Hs-mode current-source pull-up: only while this controller has released
  // SCL; on the first rising edge after Sr / ACK wait until SCL is high.
  always_comb begin : p_cs_en
    if (hs_active & own_bus & st_high)
      cs_en = scl_f | (r_bit_idx != 4'd0);
    else
      cs_en = hs_active & own_bus & (st_sr_h | st_sr_b | st_p_h | st_p_w) & scl_f;
  end

endmodule
