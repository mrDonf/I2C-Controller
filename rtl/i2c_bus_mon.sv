//-----------------------------------------------------------------------------
// i2c_bus_mon (spec 4.3)
//   - START / STOP detection
//   - bus busy / bus free (tBUF), bus-idle detection
//   - bit counter, bus error (misplaced START/STOP)
//   - Hs-mode detection (S + 0000_1xxx + NACK) and exit on STOP
//   - SCL low timeout
//-----------------------------------------------------------------------------
module i2c_bus_mon (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        en,             // CTRL.EN and not soft reset
  input  logic        busclr_active,  // controller is doing bus clear
  input  logic        scl_f,
  input  logic        sda_f,
  input  logic        scl_rise,
  input  logic        scl_fall,
  input  logic        sda_rise,
  input  logic        sda_fall,
  input  logic [15:0] tbuf,           // TIM_STO.TBUF   (0 behaves as 1)
  input  logic [15:0] idle_val,       // BUSIDLE.IDLE_VAL (0 behaves as 1)
  input  logic        tout_en,
  input  logic [23:0] tout_val,       // TIMEOUT.TOUT_VAL (0 behaves as 1)
  output logic        start_det,      // 1-cycle pulse
  output logic        stop_det,       // 1-cycle pulse
  output logic        bus_busy,
  output logic        bus_free,
  output logic        hs_active,
  output logic        hs_det,         // 1-cycle pulse
  output logic        bus_err,        // 1-cycle pulse
  output logic        scl_tout,       // 1-cycle pulse
  output logic [3:0]  bit_cnt         // SCL rising edges since START / last ACK clock
);

  import apb_i2c_pkg::min1;

  //---------------------------------------------------------------------------
  // START / STOP: SDA edge while SCL is high in this and the previous cycle
  //---------------------------------------------------------------------------
  assign start_det = en & sda_fall & scl_f & ~scl_rise;
  assign stop_det  = en & sda_rise & scl_f & ~scl_rise;

  //---------------------------------------------------------------------------
  // Bus idle counter: consecutive cycles with SCL = SDA = 1. After a STOP the
  // lines are high, so it also counts the cycles since stop_det (0 in the
  // stop_det cycle) and serves as the tBUF counter.
  //---------------------------------------------------------------------------
  logic [15:0] r_idle_cnt;
  logic        r_en_d;         // CTRL.EN delayed (detects 0 -> 1)

  always_ff @(posedge clk or negedge rst_n) begin : p_idle_cnt
    if (!rst_n)                        r_idle_cnt <= 16'd0;
    else if (!en || !(scl_f && sda_f)) r_idle_cnt <= 16'd0;
    else if (r_idle_cnt != 16'hFFFF)   r_idle_cnt <= r_idle_cnt + 16'd1;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_en_d
    if (!rst_n) r_en_d <= 1'b0;
    else        r_en_d <= en;
  end

  //---------------------------------------------------------------------------
  // Busy / tBUF
  //---------------------------------------------------------------------------
  logic r_busy;
  logic r_tbuf_done;
  logic idle_reached;
  logic tbuf_reached;

  assign idle_reached = (r_idle_cnt >= min1(idle_val));
  assign tbuf_reached = (r_idle_cnt >  min1(tbuf));

  always_ff @(posedge clk or negedge rst_n) begin : p_busy
    if (!rst_n)            r_busy <= 1'b0;
    else if (!en)          r_busy <= 1'b0;
    else if (!r_en_d)      r_busy <= 1'b1;   // just enabled: state unknown
    else if (start_det)    r_busy <= 1'b1;
    else if (stop_det)     r_busy <= 1'b0;
    else if (idle_reached) r_busy <= 1'b0;
  end

  // (start_det and stop_det are never active together)
  always_ff @(posedge clk or negedge rst_n) begin : p_tbuf
    if (!rst_n)                          r_tbuf_done <= 1'b0;
    else if (!en || !r_en_d || stop_det) r_tbuf_done <= 1'b0;
    else if (!start_det && (idle_reached || (!r_busy && tbuf_reached)))
                                               r_tbuf_done <= 1'b1;
  end

  assign bus_busy = r_busy;
  assign bus_free = ~r_busy & r_tbuf_done;

  //---------------------------------------------------------------------------
  // Bit counter and bus error
  //---------------------------------------------------------------------------
  logic [3:0] r_bit_cnt;
  logic       r_bit_valid;    // bit counter is aligned to a START
  logic       ack_clk_end;    // SCL falling edge of the 9th clock

  assign ack_clk_end = scl_fall & (r_bit_cnt == 4'd9);

  always_ff @(posedge clk or negedge rst_n) begin : p_bit_cnt
    if (!rst_n)                               r_bit_cnt <= 4'd0;
    else if (!en || busclr_active)            r_bit_cnt <= 4'd0;
    else if (start_det || stop_det)           r_bit_cnt <= 4'd0;
    else if (scl_rise && (r_bit_cnt != 4'd9)) r_bit_cnt <= r_bit_cnt + 4'd1;
    else if (ack_clk_end)                     r_bit_cnt <= 4'd0;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_bit_valid
    if (!rst_n)                    r_bit_valid <= 1'b0;
    else if (!en || busclr_active) r_bit_valid <= 1'b0;
    else if (start_det)            r_bit_valid <= 1'b1;
    else if (stop_det)             r_bit_valid <= 1'b0;
  end

  assign bus_err = ~busclr_active & r_bit_valid & (start_det | stop_det) & (r_bit_cnt >= 4'd2);
  assign bit_cnt = r_bit_cnt;

  //---------------------------------------------------------------------------
  // Hs-mode detection: the first byte after a START in F/S mode is checked
  // bit by bit against 0000_1xxx followed by NACK (bit_cnt = index of the bit
  // whose rising edge is being seen).
  //---------------------------------------------------------------------------
  logic r_hs_code;      // first byte after S still matches 0000_1xxx + NACK
  logic r_hs_active;
  logic hs_bit_ok;      // SDA value of the current bit matches the code

  always_comb begin : p_hs_bit_ok
    unique case (r_bit_cnt)
      4'd0, 4'd1, 4'd2, 4'd3: hs_bit_ok = ~sda_f;   // must be 0
      4'd4, 4'd8:             hs_bit_ok =  sda_f;   // 1, NACK
      default:                hs_bit_ok = 1'b1;     // MCODE
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_hs_code
    if (!rst_n)                        r_hs_code <= 1'b0;
    else if (!en || stop_det)          r_hs_code <= 1'b0;
    else if (start_det)                r_hs_code <= ~r_hs_active;
    else if (scl_rise && !hs_bit_ok)   r_hs_code <= 1'b0;
    else if (!scl_rise && ack_clk_end) r_hs_code <= 1'b0;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_hs_active
    if (!rst_n)               r_hs_active <= 1'b0;
    else if (!en || stop_det) r_hs_active <= 1'b0;
    else if (hs_det)          r_hs_active <= 1'b1;
  end

  assign hs_active = r_hs_active;
  // (START / STOP need SCL high, ack_clk_end SCL falling: never together)
  assign hs_det    = en & r_hs_code & ack_clk_end;

  //---------------------------------------------------------------------------
  // SCL low timeout (reported once per continuous SCL-low period).
  // r_tout_n = SCL-low cycles including the current one (1 when SCL is high),
  // so the timeout test is a direct comparison with TOUT_VAL.
  //---------------------------------------------------------------------------
  logic [23:0] r_tout_n;
  logic        r_tout_hit;     // timeout already reported for this SCL-low period
  logic        tout_clr;
  logic        tout_reached;

  assign tout_clr     = ~en | ~tout_en | scl_f;
  assign tout_reached = (r_tout_n >= tout_val);

  always_ff @(posedge clk or negedge rst_n) begin : p_tout_cnt
    if (!rst_n)                            r_tout_n <= 24'd1;
    else if (tout_clr)                     r_tout_n <= 24'd1;
    else if (!r_tout_hit && !tout_reached) r_tout_n <= r_tout_n + 24'd1;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_tout_hit
    if (!rst_n)            r_tout_hit <= 1'b0;
    else if (tout_clr)     r_tout_hit <= 1'b0;
    else if (tout_reached) r_tout_hit <= 1'b1;
  end

  assign scl_tout = ~tout_clr & ~r_tout_hit & tout_reached;

endmodule
