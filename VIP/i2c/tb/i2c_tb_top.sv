// i2c_tb_top - VIP master and two VIP slaves on one I2C bus (plus an unused APB interface for the DUT tests of the package)
`timescale 1ns/1ps
module i2c_tb_top;
  import uvm_pkg::*;
  import i2c_pkg::*;
  import apb_pkg::*;
  import i2c_test_pkg::*;

  vip_clk_rst_if crif();
  vip_i2c_if i2cif (.clk(crif.clk), .rst_n(crif.rst_n));
  vip_apb_if #(APB_ADDR_W, APB_DATA_W) apb_unused (.pclk(crif.clk), .presetn(crif.rst_n));
  initial begin apb_unused.checks_en = 1'b0; apb_unused.psel = 1'b0; apb_unused.penable = 1'b0; end

  initial begin
    uvm_config_db #(i2c_vif_t)::set(null, "*", "vif", i2cif);
    uvm_config_db #(virtual vip_clk_rst_if)::set(null, "*", "crif", crif);
    run_test();
  end
endmodule : i2c_tb_top
