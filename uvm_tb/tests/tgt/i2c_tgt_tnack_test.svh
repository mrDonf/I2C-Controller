// -----------------------------------------------------------------------------
// i2c_tgt_tnack_test  [VP-TGT-10]
//   TNACK: address ACKed, data NACKed, transmit unaffected (5.2.11)
// -----------------------------------------------------------------------------
class i2c_tgt_tnack_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_tnack_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_tnack_vseq s = i2c_tgt_tnack_vseq::type_id::create("tn");
    start_seq(s);
  endtask
endclass : i2c_tgt_tnack_test
