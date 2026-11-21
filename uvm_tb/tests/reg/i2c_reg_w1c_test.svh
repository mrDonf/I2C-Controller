// -----------------------------------------------------------------------------
// i2c_reg_w1c_test  [VP-REG-09]
//   W1C / W1SC registers, DMA_CTRL lane rule, register model mirror (5.2)
// -----------------------------------------------------------------------------
class i2c_reg_w1c_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_w1c_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_reg_w1c_w1s_vseq s = i2c_reg_w1c_w1s_vseq::type_id::create("w1c");
    start_seq(s);
  endtask
endclass : i2c_reg_w1c_test
