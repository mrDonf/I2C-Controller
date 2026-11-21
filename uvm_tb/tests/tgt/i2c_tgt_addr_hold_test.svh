// -----------------------------------------------------------------------------
// i2c_tgt_addr_hold_test  [VP-TGT-16]
//   corner: address (write / read) ACKed while TRX lacks 2 free entries: stretch until the software reads TRX,
//   then TX_REQ for a read (4.5.3, 4.5.4)
// -----------------------------------------------------------------------------
class i2c_tgt_addr_hold_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_addr_hold_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_addr_hold_vseq s = i2c_tgt_addr_hold_vseq::type_id::create("ah");
    if (!s.randomize() with { is_read == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);                          // held write address
    if (!s.randomize() with { is_read == 1; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);                          // held read address, TX_REQ afterwards
  endtask
endclass : i2c_tgt_addr_hold_test
