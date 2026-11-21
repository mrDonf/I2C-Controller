// -----------------------------------------------------------------------------
// i2c_tgt_serviced_read_test  [VP-TGT-07]
//   reads longer than TTX refilled by the TX_REQ routine; register read flow S W ptr Sr R (6.11)
// -----------------------------------------------------------------------------
class i2c_tgt_serviced_read_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_serviced_read_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_serviced_read_vseq s = i2c_tgt_serviced_read_vseq::type_id::create("srd");
    if (!s.randomize() with { reg_read == 1; n == 6; use_addr1 == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { reg_read == 0; n == 16; use_addr1 == 1; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    repeat (4) rand_start(s);
  endtask
endclass : i2c_tgt_serviced_read_test
