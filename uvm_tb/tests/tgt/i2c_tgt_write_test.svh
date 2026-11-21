// -----------------------------------------------------------------------------
// i2c_tgt_write_test  [VP-TGT-01]
//   target receiver: ADDR0 / ADDR1, STATUS / TSTAT / TRX entries / TADDR / TSTOP / irq_tgt_o, random lengths (4.5,
//   6.10)
// -----------------------------------------------------------------------------
class i2c_tgt_write_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_write_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_write_vseq w = i2c_tgt_write_vseq::type_id::create("w");
    if (!w.randomize() with { use_addr1 == 0; n == 3; }) `uvm_fatal("TEST", "randomize")
    start_seq(w);
    if (!w.randomize() with { use_addr1 == 1; n == 1; }) `uvm_fatal("TEST", "randomize")
    start_seq(w);
    repeat (6) begin
      if (!w.randomize() with { n inside {[1:16]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(w);
    end
  endtask
endclass : i2c_tgt_write_test
