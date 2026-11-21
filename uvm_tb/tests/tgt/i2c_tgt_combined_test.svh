// -----------------------------------------------------------------------------
// i2c_tgt_combined_test  [VP-TGT-17]
//   corner: combined format: read + Sr write / read, Device ID arming + Sr read, own read + Sr Device ID (4.5.2,
//   4.5.6)
// -----------------------------------------------------------------------------
class i2c_tgt_combined_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_combined_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_combined_vseq s = i2c_tgt_combined_vseq::type_id::create("comb");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_tgt_combined_test
