// -----------------------------------------------------------------------------
// i2c_ctrl_rstart_chain_test  [VP-CTL-17]
//   corner: one transaction with a chain of repeated STARTs to several targets, read-back through Sr
// -----------------------------------------------------------------------------
class i2c_ctrl_rstart_chain_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_rstart_chain_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_rstart_chain_vseq ch = i2c_ctrl_rstart_chain_vseq::type_id::create("ch");
    repeat (5) rand_start(ch);
  endtask
endclass : i2c_ctrl_rstart_chain_test
