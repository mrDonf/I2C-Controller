// apb_dut_tb_top - the APB master VIP drives the GPIO IP of this repository (real DUT as completer)
`timescale 1ns/1ps
module apb_dut_tb_top;
  import uvm_pkg::*;
  import apb_pkg::*;
  import apb_test_pkg::*;

  vip_clk_rst_if crif();
  vip_apb_if #(APB_ADDR_W, APB_DATA_W) vif (.pclk(crif.clk), .presetn(crif.rst_n));

  wire [31:0] pad;
  logic [31:0] g_o, g_oe, g_pen, g_pup;
  logic irq;

  gpio #(.WIDTH(32)) dut (
    .pclk(crif.clk), .presetn(crif.rst_n),
    .psel(vif.psel), .penable(vif.penable), .pwrite(vif.pwrite), .paddr(vif.paddr[11:0]),
    .pwdata(vif.pwdata), .pstrb(vif.pstrb), .pready(vif.pready), .prdata(vif.prdata), .pslverr(vif.pslverr),
    .irq(irq), .gpio_i(pad), .gpio_o(g_o), .gpio_oe(g_oe), .gpio_pull_en(g_pen), .gpio_pull_up(g_pup));

  generate
    for (genvar i = 0; i < 32; i++) begin : g_pad
      assign pad[i] = g_oe[i] ? g_o[i] : 1'bz;
      assign (weak1, weak0) pad[i] = g_pen[i] ? g_pup[i] : 1'b0;
    end
  endgenerate

  initial begin
    uvm_config_db #(apb_vif_t)::set(null, "*", "vif", vif);
    uvm_config_db #(virtual vip_clk_rst_if)::set(null, "*", "crif", crif);
    run_test();
  end
endmodule : apb_dut_tb_top
