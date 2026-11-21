// -----------------------------------------------------------------------------
// i2c_ctrl_gcall_test  [VP-CTL-15]
//   general call with / without NAKOK, Device ID read of an external device (6.8, 6.9)
// -----------------------------------------------------------------------------
class i2c_ctrl_gcall_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_gcall_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_gcall_vseq gc = i2c_ctrl_gcall_vseq::type_id::create("gc");
    start_seq(gc);
  endtask
endclass : i2c_ctrl_gcall_test
