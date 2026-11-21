// -----------------------------------------------------------------------------
// i2c_reg_hw_reset_test  [VP-REG-01]
//   reset values: UVM uvm_reg_hw_reset_seq + explicit spec table + output ports after reset (5.1, 6.1.1)
// -----------------------------------------------------------------------------
class i2c_reg_hw_reset_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_hw_reset_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_reset_outputs_vseq outs = i2c_reset_outputs_vseq::type_id::create("outs");
    uvm_reg_hw_reset_seq   rst  = uvm_reg_hw_reset_seq::type_id::create("rst");
    i2c_reg_reset_val_vseq vals = i2c_reg_reset_val_vseq::type_id::create("vals");
    start_seq(outs);                       // ports first, before any register access
    rst.model = env.ral;                   // standard UVM register sequence (front door through the APB VIP)
    rst.start(null);
    start_seq(vals);                       // spec table, independent of the register model
  endtask
endclass : i2c_reg_hw_reset_test
