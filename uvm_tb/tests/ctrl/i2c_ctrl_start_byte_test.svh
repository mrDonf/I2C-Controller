// -----------------------------------------------------------------------------
// i2c_ctrl_start_byte_test  [VP-CTL-14]
//   START byte prefix on every START from idle, not on Sr (4.4.8, 6.7)
// -----------------------------------------------------------------------------
class i2c_ctrl_start_byte_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_start_byte_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_start_byte_vseq sb = i2c_ctrl_start_byte_vseq::type_id::create("sb");
    i2c_ip_ctrl_xfer_vseq    x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(sb);
    rand_start(x);                         // STARTBYTE_EN = 0 again: no prefix
  endtask
endclass : i2c_ctrl_start_byte_test
