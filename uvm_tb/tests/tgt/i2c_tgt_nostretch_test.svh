// -----------------------------------------------------------------------------
// i2c_tgt_nostretch_test  [VP-TGT-09]
//   NOSTRETCH: data / address NACK with TRX full + TRX_OVF, TTX underflow 0xFF + TTX_UNF, never stretches (4.5.3,
//   4.5.5)
// -----------------------------------------------------------------------------
class i2c_tgt_nostretch_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_nostretch_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_tcfg = 4'h2;                              // NOSTRETCH
  endfunction

  virtual task run_scenario();
    i2c_tgt_nostretch_rx_vseq rx = i2c_tgt_nostretch_rx_vseq::type_id::create("rx");
    i2c_tgt_nostretch_tx_vseq tx = i2c_tgt_nostretch_tx_vseq::type_id::create("tx");
    start_seq(rx);
    start_seq(tx);
  endtask
endclass : i2c_tgt_nostretch_test
