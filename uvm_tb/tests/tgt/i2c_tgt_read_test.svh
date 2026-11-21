// -----------------------------------------------------------------------------
// i2c_tgt_read_test  [VP-TGT-05]
//   target transmitter with preloaded TTX, TTX_DONE, partial read, leftover served next, TTX_FLUSH (4.5.4, 6.11)
// -----------------------------------------------------------------------------
class i2c_tgt_read_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_read_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_read_vseq s = i2c_tgt_read_vseq::type_id::create("rd");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_tgt_read_test
