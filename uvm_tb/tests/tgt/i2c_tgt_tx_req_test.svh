// -----------------------------------------------------------------------------
// i2c_tgt_tx_req_test  [VP-TGT-06]
//   TTX empty: clock stretching after the address ACK, one TX_REQ, data after the software load (8.5)
// -----------------------------------------------------------------------------
class i2c_tgt_tx_req_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_tx_req_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_tx_req_vseq s = i2c_tgt_tx_req_vseq::type_id::create("txr");
    start_seq(s);
    repeat (3) begin
      if (!s.randomize() with { n inside {[1:local::depth]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(s);
    end
  endtask
endclass : i2c_tgt_tx_req_test
