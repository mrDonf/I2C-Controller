//-----------------------------------------------------------------------------
// i2c_in_filter
//   2-FF synchronizer + glitch (spike) filter + edge detection (spec 4.2).
//   Pulses of width <= spklen PCLK cycles are suppressed.
//   Latency pad -> dout : spklen + 3 cycles.
//-----------------------------------------------------------------------------
module i2c_in_filter (
  input  logic       clk,
  input  logic       rst_n,
  input  logic       din,      // asynchronous pad input
  input  logic [7:0] spklen,
  output logic       dout,     // filtered level
  output logic       rise,     // 1-cycle pulse: dout 0 -> 1
  output logic       fall      // 1-cycle pulse: dout 1 -> 0
);

  //---------------------------------------------------------------------------
  // Synchronizer (reset value 1 = released line)
  //---------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic r_sync1;
  (* ASYNC_REG = "TRUE" *) logic r_sync2;

  always_ff @(posedge clk or negedge rst_n) begin : p_sync
    if (!rst_n) begin
      r_sync1 <= 1'b1;
      r_sync2 <= 1'b1;
    end else begin
      r_sync1 <= din;
      r_sync2 <= r_sync1;
    end
  end

  //---------------------------------------------------------------------------
  // Spike filter: the output follows the input once it has differed for more
  // than spklen consecutive cycles
  //---------------------------------------------------------------------------
  logic       r_filt;
  logic [7:0] r_cnt;           // cycles the synchronized input differs from r_filt
  logic       differ;
  logic       accept;

  assign differ = (r_sync2 != r_filt);
  assign accept = differ & (r_cnt >= spklen);

  always_ff @(posedge clk or negedge rst_n) begin : p_cnt
    if (!rst_n)                 r_cnt <= 8'd0;
    else if (!differ || accept) r_cnt <= 8'd0;
    else                        r_cnt <= r_cnt + 8'd1;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_filt
    if (!rst_n)      r_filt <= 1'b1;
    else if (accept) r_filt <= r_sync2;
  end

  //---------------------------------------------------------------------------
  // Edges
  //---------------------------------------------------------------------------
  logic r_filt_d;

  always_ff @(posedge clk or negedge rst_n) begin : p_filt_d
    if (!rst_n) r_filt_d <= 1'b1;
    else        r_filt_d <= r_filt;
  end

  assign dout   = r_filt;
  assign rise   =  r_filt & ~r_filt_d;
  assign fall   = ~r_filt &  r_filt_d;

endmodule
