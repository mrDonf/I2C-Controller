// i2c_dut_tb_top - APB VIP + I2C IP of this repository (master) + I2C VIP slave (EEPROM model at 0x50)
`timescale 1ns/1ps
module i2c_dut_tb_top;
  import uvm_pkg::*;
  import apb_pkg::*;
  import i2c_pkg::*;
  import i2c_test_pkg::*;

  vip_clk_rst_if crif();
  initial crif.period_ps = 20000;
  vip_apb_if #(APB_ADDR_W, APB_DATA_W) apb (.pclk(crif.clk), .presetn(crif.rst_n));
  vip_i2c_if i2cif (.clk(crif.clk), .rst_n(crif.rst_n));
  logic irq, scl_oe, sda_oe;

  // the IP's open-drain outputs join the bus nets of the VIP interface
  assign i2cif.scl = scl_oe ? 1'b0 : 1'bz;
  assign i2cif.sda = sda_oe ? 1'b0 : 1'bz;

  i2c #(.FIFO_DEPTH(16)) dut (
    .pclk(crif.clk), .presetn(crif.rst_n),
    .psel(apb.psel), .penable(apb.penable), .pwrite(apb.pwrite), .paddr(apb.paddr[11:0]),
    .pwdata(apb.pwdata), .pstrb(apb.pstrb), .pready(apb.pready), .prdata(apb.prdata), .pslverr(apb.pslverr),
    .irq(irq), .scl_i(i2cif.scl), .sda_i(i2cif.sda), .scl_oe(scl_oe), .sda_oe(sda_oe));

  initial begin
    uvm_config_db #(apb_vif_t)::set(null, "*", "apb_vif", apb);
    uvm_config_db #(i2c_vif_t)::set(null, "*", "vif", i2cif);
    uvm_config_db #(virtual vip_clk_rst_if)::set(null, "*", "crif", crif);
    run_test();
  end
endmodule : i2c_dut_tb_top
