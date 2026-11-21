// -----------------------------------------------------------------------------
// i2c_ip_dma_if - DMA request / acknowledge channel of apb_i2c (spec 3.4, 4.9)
//   req / req_type : IP -> DMAC      ack / ack_type : DMAC -> IP (driven by the DMAC model of the TB)
// -----------------------------------------------------------------------------
`ifndef I2C_IP_DMA_IF_SV
`define I2C_IP_DMA_IF_SV

interface i2c_ip_dma_if (input logic clk, input logic rst_n);
  logic       req;
  logic [1:0] req_type;
  logic       ack      = 1'b0;
  logic [1:0] ack_type = 2'b00;

  clocking drv_cb @(posedge clk);
    default input #1step output #1;
    input  req, req_type;
    output ack, ack_type;
  endclocking

  clocking mon_cb @(posedge clk);
    default input #1step;
    input req, req_type, ack, ack_type;
  endclocking
endinterface : i2c_ip_dma_if

`endif
