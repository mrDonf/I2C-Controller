// -----------------------------------------------------------------------------
// tb_top - UVM testbench top of apb_i2c
//
//   PCLK = 100 MHz (T = 10 ns, the reference clock of the spec examples), PRESETn from vip_clk_rst_if
//   APB  : APB VIP interface (requester = CPU model + DMAC model)            -> DUT APB4 slave port
//   I2C  : I2C VIP interface (tri1 SCL/SDA, VIP master / slaves)            <-> DUT open-drain pads
//          + bit-bang / fault-injection pull-downs (i2c_bb_if)
//   DMA  : i2c_ip_dma_if (DMAC model)                                       <-> DUT dma_req / dma_ack / dma_end
//   side : i2c_ip_side_if (irq_*, hs_mode_o, scl_cs_en_o, debug_* status)   <-  DUT (+ bus lines)
//   qch  : i2c_ip_qch_if (LPI clock controller model)                       <-> DUT QREQn / QACCEPTn / QDENY / QACTIVE;
//          the DUT PCLK is the free-running clock gated by the ICG of the interface (APB / DMAC / VIPs stay on
//          the free-running clock, like an APB bridge and a DMAC outside the gated domain)
//   SVA  : i2c_ip_sva on the DUT ports
//
//   Compile-time knobs: +define+FIFO_DEPTH=<4|8|16|32> (default 16), +define+QREQN_RST=<0|1> (default 1)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

`ifndef FIFO_DEPTH
  `define FIFO_DEPTH 16
`endif
`ifndef QREQN_RST
  `define QREQN_RST 1
`endif

module tb_top;
  import uvm_pkg::*;
  import apb_pkg::*;
  import i2c_pkg::*;
  import i2c_qch_pkg::*;
  import i2c_ip_env_pkg::*;
  import i2c_ip_test_pkg::*;

  localparam int          P_FIFO_DEPTH = `FIFO_DEPTH;
  localparam logic [23:0] P_DEVID_RST  = 24'h7A35C1;
  localparam logic [31:0] P_VERSION    = 32'h0002_0200;
  localparam logic        P_QREQN_RST  = 1'(`QREQN_RST);

  // ---------------------------------------------------------------- clock / reset
  vip_clk_rst_if crif ();
  initial begin
    crif.period_ps   = 10000;
    crif.init_cycles = 10;
  end

  // ---------------------------------------------------------------- interfaces
  vip_apb_if #(APB_ADDR_W, APB_DATA_W) apbif (.pclk(crif.clk), .presetn(crif.rst_n));
  vip_i2c_if     i2cif  (.clk(crif.clk), .rst_n(crif.rst_n));
  i2c_bb_if      bbif   (.clk(crif.clk), .rst_n(crif.rst_n));
  i2c_ip_dma_if  dmaif  (.clk(crif.clk), .rst_n(crif.rst_n));
  i2c_ip_side_if sideif (.clk(crif.clk), .rst_n(crif.rst_n));
  i2c_ip_qch_if  qchif  (.clk(crif.clk), .rst_n(crif.rst_n));

  initial qchif.qreqn = P_QREQN_RST;                                 // LPI controller in reset

  // ---------------------------------------------------------------- open-drain bus (spec 3.2 pad example)
  logic scl_oe, sda_oe;
  assign i2cif.scl = scl_oe      ? 1'b0 : 1'bz;
  assign i2cif.sda = sda_oe      ? 1'b0 : 1'bz;
  assign i2cif.scl = bbif.scl_oe ? 1'b0 : 1'bz;
  assign i2cif.sda = bbif.sda_oe ? 1'b0 : 1'bz;

  // ---------------------------------------------------------------- DUT
  apb_i2c #(
    .FIFO_DEPTH (P_FIFO_DEPTH),
    .APB_AW     (12),
    .DEVID_RST  (P_DEVID_RST),
    .VERSION    (P_VERSION),
    .QREQN_RST  (P_QREQN_RST)
  ) dut (
    .PCLK                (qchif.gclk),
    .PRESETn             (crif.rst_n),
    .PSEL                (apbif.psel),
    .PENABLE             (apbif.penable),
    .PWRITE              (apbif.pwrite),
    .PADDR               (apbif.paddr[11:0]),
    .PWDATA              (apbif.pwdata),
    .PSTRB               (apbif.pstrb),
    .PPROT               (apbif.pprot),
    .PRDATA              (apbif.prdata),
    .PREADY              (apbif.pready),
    .PSLVERR             (apbif.pslverr),
    .scl_i               (i2cif.scl),
    .scl_oe_o            (scl_oe),
    .sda_i               (i2cif.sda),
    .sda_oe_o            (sda_oe),
    .scl_cs_en_o         (sideif.cs_en),
    .hs_mode_o           (sideif.hs_mode),
    .irq_o               (sideif.irq),
    .irq_ctrl_o          (sideif.irq_ctrl),
    .irq_tgt_o           (sideif.irq_tgt),
    .irq_err_o           (sideif.irq_err),
    .dma_req_o           (dmaif.req),
    .dma_req_type_o      (dmaif.req_type),
    .dma_ack_i           (dmaif.ack),
    .dma_ack_type_i      (dmaif.ack_type),
    .dma_end_req_i       (dmaif.end_req),
    .dma_end_ack_o       (dmaif.end_ack),
    .debug_bus_busy_o    (sideif.d_bus_busy),
    .debug_ctrl_active_o (sideif.d_ctrl_active),
    .debug_ctrl_hold_o   (sideif.d_ctrl_hold),
    .debug_tgt_active_o  (sideif.d_tgt_active),
    .debug_tgt_hold_o    (sideif.d_tgt_hold),
    .debug_dma_sel_o     (sideif.d_dma_sel),
    .debug_intr_stat_o   (sideif.d_intr_stat),
    .QREQn               (qchif.qreqn),
    .QACCEPTn            (qchif.qacceptn),
    .QDENY               (qchif.qdeny),
    .QACTIVE             (qchif.qactive)
  );

  assign sideif.scl_oe  = scl_oe;
  assign sideif.sda_oe  = sda_oe;
  assign sideif.bus_scl = i2cif.scl;
  assign sideif.bus_sda = i2cif.sda;

  // Q-Channel observation (wake source classification / idle check of the controller model)
  assign qchif.o_psel        = apbif.psel;
  assign qchif.o_scl         = i2cif.scl;
  assign qchif.o_sda         = i2cif.sda;
  assign qchif.o_end_req     = dmaif.end_req;
  assign qchif.o_dma_req     = dmaif.req;
  assign qchif.o_bus_busy    = sideif.d_bus_busy;
  assign qchif.o_ctrl_active = sideif.d_ctrl_active;
  assign qchif.o_tgt_active  = sideif.d_tgt_active;

  // ---------------------------------------------------------------- port-level assertions
  i2c_ip_sva #(.QREQN_RST(P_QREQN_RST)) u_sva (
    .PCLK (qchif.gclk), .PRESETn (crif.rst_n),
    .PSEL (apbif.psel), .PENABLE (apbif.penable), .PWRITE (apbif.pwrite), .PADDR (apbif.paddr[11:0]),
    .PWDATA (apbif.pwdata), .PSTRB (apbif.pstrb), .PREADY (apbif.pready), .PSLVERR (apbif.pslverr), .PRDATA (apbif.prdata),
    .scl_oe_o (scl_oe), .sda_oe_o (sda_oe), .scl_cs_en_o (sideif.cs_en), .hs_mode_o (sideif.hs_mode),
    .irq_o (sideif.irq), .irq_ctrl_o (sideif.irq_ctrl), .irq_tgt_o (sideif.irq_tgt), .irq_err_o (sideif.irq_err),
    .dma_req_o (dmaif.req), .dma_req_type_o (dmaif.req_type), .dma_ack_i (dmaif.ack), .dma_ack_type_i (dmaif.ack_type),
    .dma_end_req_i (dmaif.end_req), .dma_end_ack_o (dmaif.end_ack),
    .debug_intr_stat_o (sideif.d_intr_stat), .debug_bus_busy_o (sideif.d_bus_busy),
    .debug_ctrl_active_o (sideif.d_ctrl_active), .debug_ctrl_hold_o (sideif.d_ctrl_hold),
    .debug_tgt_active_o (sideif.d_tgt_active), .debug_tgt_hold_o (sideif.d_tgt_hold), .debug_dma_sel_o (sideif.d_dma_sel),
    .QREQn (qchif.qreqn), .QACCEPTn (qchif.qacceptn), .QDENY (qchif.qdeny), .QACTIVE (qchif.qactive)
  );

  // ---------------------------------------------------------------- UVM
  initial begin
    automatic i2c_ip_tb_params p = new();
    p.fifo_depth = P_FIFO_DEPTH;
    p.devid_rst  = P_DEVID_RST;
    p.version    = P_VERSION;
    p.qreqn_rst  = P_QREQN_RST;
    p.pclk_ns    = 10.0;
    uvm_config_db #(i2c_ip_tb_params)::set(null, "*", "tb_params", p);
    uvm_config_db #(apb_vif_t)::set(null, "*", "apb_vif", apbif);
    uvm_config_db #(i2c_vif_t)::set(null, "*", "i2c_vif", i2cif);
    uvm_config_db #(virtual i2c_bb_if)::set(null, "*", "bb_vif", bbif);
    uvm_config_db #(virtual i2c_ip_dma_if)::set(null, "*", "dma_vif", dmaif);
    uvm_config_db #(virtual i2c_ip_side_if)::set(null, "*", "side_vif", sideif);
    uvm_config_db #(virtual i2c_ip_qch_if)::set(null, "*", "qch_vif", qchif);
    uvm_config_db #(virtual vip_clk_rst_if)::set(null, "*", "crif", crif);
    run_test();
  end
endmodule : tb_top
