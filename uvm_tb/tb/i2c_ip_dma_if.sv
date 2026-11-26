// -----------------------------------------------------------------------------
// i2c_ip_dma_if - DMA channel of apb_i2c (spec 3.4, 4.9)
//   request : req / req_type (IP -> DMAC), ack / ack_type (DMAC -> IP)   (= DMA-350 trigger input)
//   end     : end_req (DMAC -> IP), end_ack (IP -> DMAC)                  (= DMA-350 trigger output)
//   ack / ack_type / end_req are driven by the DMAC model of the TB.
//   hs_busy : TB model of DMA_STAT.BUSY (request handshake not IDLE, spec 4.9.3) from the port signals:
//             REQ <=> req = 1; ACK is entered when ack = 1 is sampled (also late, in IDLE) or when the request is
//             withdrawn (DMA_CTRL.CLR) and left when ack = 0 is sampled => BUSY = req | req(-1) | ack(-1).
// -----------------------------------------------------------------------------
`ifndef I2C_IP_DMA_IF_SV
`define I2C_IP_DMA_IF_SV

interface i2c_ip_dma_if (input logic clk, input logic rst_n);
  logic       req;
  logic [1:0] req_type;
  logic       ack      = 1'b0;
  logic [1:0] ack_type = 2'b00;
  logic       end_req  = 1'b0;
  logic       end_ack;
  logic       req_d = 1'b0, ack_d = 1'b0;
  logic       hs_busy;

  always @(posedge clk) begin
    req_d <= (rst_n === 1'b1) && (req === 1'b1);
    ack_d <= (rst_n === 1'b1) && (ack === 1'b1);
  end
  assign hs_busy = (req === 1'b1) || req_d || ack_d;

  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    input  req, req_type, end_ack;
    output ack, ack_type, end_req;
  endclocking

  clocking mon_cb @(posedge clk);
    default input #1step;
    input req, req_type, ack, ack_type, end_req, end_ack;
  endclocking
endinterface : i2c_ip_dma_if

`endif
