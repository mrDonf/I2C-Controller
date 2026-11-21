// -----------------------------------------------------------------------------
// i2c_reg_strobe_test  [VP-REG-04]
//   PSTRB byte lanes on RW, W1C and FIFO push registers (3.6)
// -----------------------------------------------------------------------------
class i2c_reg_strobe_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_strobe_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_reg_strobe_vseq s = i2c_reg_strobe_vseq::type_id::create("strb");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_reg_strobe_test
