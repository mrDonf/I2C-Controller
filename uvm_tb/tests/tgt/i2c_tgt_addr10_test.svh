// -----------------------------------------------------------------------------
// i2c_tgt_addr10_test  [VP-TGT-13]
//   10-bit addressing not supported (1.4): headers refused, normal traffic right after
// -----------------------------------------------------------------------------
class i2c_tgt_addr10_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_addr10_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_addr10_vseq s = i2c_tgt_addr10_vseq::type_id::create("a10");
    start_seq(s);
  endtask
endclass : i2c_tgt_addr10_test
