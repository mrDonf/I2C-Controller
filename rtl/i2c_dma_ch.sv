//-----------------------------------------------------------------------------
// i2c_dma_ch
//   One DMA channel (spec 3.4, 4.9). Two 4-phase handshakes:
//
//   Request (= Arm DMA-350 trigger input), the IP is the requester:
//     peripheral -> DMAC : req,  req_type[1:0]
//     DMAC -> peripheral : ack,  ack_type[1:0]
//
//     req_type : 00 SINGLE, 01 LAST_SINGLE, 10 BLOCK, 11 LAST_BLOCK
//     ack_type : 00 OKAY,   01 DENY,        10 LAST_OKAY, 11 reserved (= OKAY)
//
//     req=1 (req_type stable) -> ack=1 (ack_type valid, all data transfers of
//     the request completed) -> req=0 -> ack=0 -> next request.
//     The IP never withdraws a request by itself; only the SW command clr
//     (DMA_CTRL.CLR, spec 4.9.7) drops a pending request without ack. A late
//     ack seen in IDLE is waited out in ACK without processing ack_type.
//
//   End (= DMA-350 trigger output), the DMAC is the requester (spec 4.9.6):
//     DMAC -> peripheral : end_req      peripheral -> DMAC : end_ack
//     end_ack = end_req delayed one cycle; every 0 -> 1 of end_req (sampled
//     with end_ack = 0) is one end event. A 1-cycle pulse is also accepted.
//
//   avail  : RX channel = entries in FIFO, TX channel = free entries.
//   lpos   : RX channel, 1-based position of the first LAST-marked entry
//            (0 = none). Tie to 0 for TX channels.
//   blk    : block size (0 = no BLOCK requests), <= FIFO depth.
//   pfc    : peripheral flow control (RX channels only, else tie to 0).
//   end_en : an end event completes the channel (while en = 1).
//-----------------------------------------------------------------------------
module i2c_dma_ch
  import apb_i2c_pkg::*;
(
  input  logic       clk,
  input  logic       rst_n,
  input  logic       en,
  input  logic       pfc,
  input  logic       end_en,
  input  logic       clr,        // SW command: withdraw a pending request
  input  logic [5:0] blk,
  input  logic [5:0] avail,
  input  logic [5:0] lpos,
  // request handshake
  output logic       req_o,
  output logic [1:0] req_type_o,
  input  logic       ack_i,
  input  logic [1:0] ack_type_i,
  // end handshake
  input  logic       end_req_i,
  output logic       end_ack_o,
  // status
  output logic       done,       // 1-cycle pulse: DMA operation finished
  output logic       deny,       // 1-cycle pulse: request denied
  output logic       end_ev,     // 1-cycle pulse: end event from the DMAC
  output dma_state_t state       // 0 IDLE, 1 REQ, 2 ACK
);

  dma_state_t r_state;            // state[0] is the request flop itself
  logic [1:0] r_req_type;
  logic [5:0] r_deny_avail;       // avail when the last DENY was received, 0 = none
  logic       r_end_ack;

  //---------------------------------------------------------------------------
  // Request type selection (spec 4.9.2)
  //---------------------------------------------------------------------------
  logic       want;
  logic [1:0] want_type;
  logic       blk_en;

  assign blk_en = (blk != 6'd0);

  always_comb begin : p_want
    want      = 1'b0;
    want_type = RT_SINGLE;
    if (pfc && (lpos != 6'd0)) begin
      // end of the peripheral-controlled transfer is in the FIFO
      want = 1'b1;
      if (lpos == 6'd1)                 want_type = RT_LAST_SINGLE;
      else if (blk_en && (lpos == blk)) want_type = RT_LAST_BLOCK;
      else if (blk_en && (lpos >  blk)) want_type = RT_BLOCK;
      else                              want_type = RT_SINGLE;
    end else if (blk_en && (avail >= blk)) begin
      want      = 1'b1;
      want_type = RT_BLOCK;
    end else if ((avail != 6'd0) && !(pfc && blk_en)) begin
      // peripheral flow control with blocks: no SINGLE until the LAST entry
      want      = 1'b1;
      want_type = RT_SINGLE;
    end
  end

  //---------------------------------------------------------------------------
  // Handshake decisions (spec 4.9.3, 4.9.7)
  //   After a DENY no new request until avail changes. A request always needs
  //   avail >= 1, so r_deny_avail = 0 safely encodes "not blocked".
  //   A sampled ack has priority over clr in the same cycle.
  //---------------------------------------------------------------------------
  logic blocked;
  logic issue;        // IDLE -> REQ
  logic ack_seen;     // REQ  -> ACK, ack_type processed

  assign blocked  = (avail == r_deny_avail);
  assign issue    = (r_state == D_IDLE) & ~ack_i & en & want & ~blocked;
  assign ack_seen = (r_state == D_REQ) & ack_i;

  //---------------------------------------------------------------------------
  // Handshake FSM
  //---------------------------------------------------------------------------
  dma_state_t state_nxt;

  always_comb begin : p_state_nxt
    state_nxt = r_state;
    unique case (r_state)
      D_IDLE:
        if (ack_i)              state_nxt = D_ACK;      // late ack of a withdrawn request
        else if (issue)         state_nxt = D_REQ;
      D_REQ:
        if (ack_i || clr)       state_nxt = D_ACK;      // acknowledged, or withdrawn by SW
      D_ACK:
        if (!ack_i)             state_nxt = D_IDLE;
      default:                  state_nxt = D_IDLE;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_state
    if (!rst_n) r_state <= D_IDLE;
    else        r_state <= state_nxt;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_req_type
    if (!rst_n)     r_req_type <= RT_SINGLE;
    else if (issue) r_req_type <= want_type;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_deny_avail
    if (!rst_n)
      r_deny_avail <= 6'd0;
    else if (ack_seen)
      r_deny_avail <= (ack_type_i == AT_DENY) ? avail : 6'd0;
    else if ((r_state == D_IDLE) && (!en || !blocked))
      r_deny_avail <= 6'd0;
  end

  //---------------------------------------------------------------------------
  // End handshake (spec 4.9.6): acknowledged immediately, never stalls the DMAC
  //---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin : p_end_ack
    if (!rst_n) r_end_ack <= 1'b0;
    else        r_end_ack <= end_req_i;
  end

  //---------------------------------------------------------------------------
  // Outputs
  //---------------------------------------------------------------------------
  logic ack_done;     // completion reported on the request handshake

  assign ack_done   = ack_seen & (ack_type_i != AT_DENY) &
                      (r_req_type[0] | (ack_type_i == AT_LAST_OKAY));
  assign end_ev     = end_req_i & ~r_end_ack;
  assign deny       = ack_seen & (ack_type_i == AT_DENY);
  assign done       = ack_done | (end_ev & end_en & en);
  assign req_o      = r_state[0];
  assign req_type_o = r_req_type;
  assign end_ack_o  = r_end_ack;
  assign state      = r_state;

endmodule
