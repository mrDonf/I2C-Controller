// -----------------------------------------------------------------------------
// i2c_tgt_trx_queue_test  [VP-TGT-04]
//   several transactions queued in TRX, TRX_THR interrupt (6.10)
// -----------------------------------------------------------------------------
class i2c_tgt_trx_queue_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_trx_queue_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_trx_queue_vseq s = i2c_tgt_trx_queue_vseq::type_id::create("tq");
    start_seq(s);
  endtask
endclass : i2c_tgt_trx_queue_test
