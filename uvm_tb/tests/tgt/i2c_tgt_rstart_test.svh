// -----------------------------------------------------------------------------
// i2c_tgt_rstart_test  [VP-TGT-02]
//   repeated START TRX tags: Sr between own addresses, Sr after a foreign address, write-read (4.5.2)
// -----------------------------------------------------------------------------
class i2c_tgt_rstart_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_rstart_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_rstart_vseq s = i2c_tgt_rstart_vseq::type_id::create("rs");
    start_seq(s);
  endtask
endclass : i2c_tgt_rstart_test
