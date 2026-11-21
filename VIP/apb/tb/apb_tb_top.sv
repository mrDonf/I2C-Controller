// apb_tb_top - master agent <-> slave agent (memory) on one vip_apb_if; no DUT
`timescale 1ns/1ps
module apb_tb_top;
  import uvm_pkg::*;
  import apb_pkg::*;
  import apb_test_pkg::*;

  vip_clk_rst_if crif();
  vip_apb_if #(APB_ADDR_W, APB_DATA_W) vif (.pclk(crif.clk), .presetn(crif.rst_n));

  initial begin
    uvm_config_db #(apb_vif_t)::set(null, "*", "vif", vif);
    uvm_config_db #(virtual vip_clk_rst_if)::set(null, "*", "crif", crif);
    run_test();
  end
endmodule : apb_tb_top
