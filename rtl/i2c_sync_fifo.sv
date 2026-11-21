//-----------------------------------------------------------------------------
// i2c_sync_fifo
//   Synchronous FIFO, show-ahead read data, synchronous flush.
//   - data buffer: plain 1-write / 1-read memory (no reset, accessed only at
//     the write and read pointers) -> maps to SRAM / RAM
//   - write / read pointers and the optional mark bits are flip-flops
//   - push when full and pop when empty are ignored
//   - flush has priority over push/pop (pointers); a buffer write in the
//     flush cycle is harmless (the entry is not valid afterwards), so the
//     buffer write enable does not depend on flush
//   - level is zero-extended to 6 bits (DEPTH <= 32)
//   - MARK = 1: every entry carries a mark bit (wmark); mpos returns the
//     1-based position (from the head) of the first marked entry among the
//     valid ones, 0 = none. Used by the DMA channel for LAST requests.
//-----------------------------------------------------------------------------
module i2c_sync_fifo #(
  parameter int WIDTH = 8,
  parameter int DEPTH = 16,
  parameter bit MARK  = 1'b0
) (
  input  logic             clk,
  input  logic             rst_n,
  input  logic             flush,
  input  logic             push,
  input  logic [WIDTH-1:0] wdata,
  input  logic             wmark,
  input  logic             pop,
  output logic [WIDTH-1:0] rdata,
  output logic [5:0]       level,
  output logic             full,
  output logic             empty,
  output logic [5:0]       mpos
);

  localparam int AW = $clog2(DEPTH);

  logic [WIDTH-1:0] mem [DEPTH];     // data buffer (RAM)
  logic [AW:0]      r_wptr;          // one extra wrap bit to tell full from empty
  logic [AW:0]      r_rptr;
  logic [AW:0]      count;
  logic             wr_en;           // buffer / mark write
  logic             rd_en;

  assign wr_en = push & ~full;
  assign rd_en = pop  & ~empty;

  //---------------------------------------------------------------------------
  // Pointers
  //---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin : p_wptr
    if (!rst_n)     r_wptr <= '0;
    else if (flush) r_wptr <= '0;
    else if (wr_en) r_wptr <= r_wptr + 1'b1;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_rptr
    if (!rst_n)     r_rptr <= '0;
    else if (flush) r_rptr <= '0;
    else if (rd_en) r_rptr <= r_rptr + 1'b1;
  end

  // count is computed at pointer width (modulo 2*DEPTH), then zero-extended
  assign count = r_wptr - r_rptr;
  assign level = 6'(count);
  assign empty = (r_wptr == r_rptr);
  assign full  = (r_wptr[AW] != r_rptr[AW]) && (r_wptr[AW-1:0] == r_rptr[AW-1:0]);

  //---------------------------------------------------------------------------
  // Data buffer: write port at wptr, read port at rptr
  //---------------------------------------------------------------------------
  always_ff @(posedge clk) begin : p_mem
    if (wr_en) mem[r_wptr[AW-1:0]] <= wdata;
  end

  assign rdata = mem[r_rptr[AW-1:0]];

  //---------------------------------------------------------------------------
  // Mark bits (flip-flops) and first marked entry (smallest distance from head)
  //---------------------------------------------------------------------------
  if (MARK) begin : g_mark
    logic [DEPTH-1:0] r_mark;

    always_ff @(posedge clk or negedge rst_n) begin : p_mark
      if (!rst_n) r_mark <= '0;
      else if (wr_en) r_mark[r_wptr[AW-1:0]] <= wmark;
    end

    function automatic logic [5:0] first_mark(input logic [DEPTH-1:0] mk,
                                              input logic [AW-1:0]    rp,
                                              input logic [5:0]       lvl);
      logic [AW-1:0] idx;
      first_mark = 6'd0;
      for (int i = DEPTH - 1; i >= 0; i--) begin
        idx = rp + AW'(i);
        if ((6'(i) < lvl) && mk[idx]) first_mark = 6'(i + 1);
      end
    endfunction

    assign mpos = first_mark(r_mark, r_rptr[AW-1:0], level);
  end else begin : g_nomark
    assign mpos = 6'd0;
  end

endmodule
