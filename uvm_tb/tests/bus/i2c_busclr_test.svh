// -----------------------------------------------------------------------------
// i2c_busclr_test  [VP-BUS-05]
//   bus clear: without CTRL_EN, while the controller is busy, SDA released after k clocks, SDA stuck, SCL stuck +
//   timeout (4.4.10, 6.14)
// -----------------------------------------------------------------------------
class i2c_busclr_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_busclr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_busclr_vseq       bc = i2c_busclr_vseq::type_id::create("bc");
    i2c_ip_ctrl_xfer_vseq x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    i2c_ip_busclr_e kinds[$] = '{BC_NO_CTRL_EN, BC_SDA_RELEASE, BC_SDA_STUCK, BC_SCL_STUCK, BC_WHILE_BUSY, BC_SDA_RELEASE};
    foreach (kinds[i]) begin
      if (!bc.randomize() with { kind == local::kinds[i]; }) `uvm_fatal("TEST", "randomize")
      start_seq(bc);
    end
    rand_start(x);                         // normal operation afterwards
  endtask
endclass : i2c_busclr_test
