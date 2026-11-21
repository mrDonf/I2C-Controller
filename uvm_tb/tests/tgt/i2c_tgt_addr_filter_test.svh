// -----------------------------------------------------------------------------
// i2c_tgt_addr_filter_test  [VP-TGT-03]
//   address filter: foreign address, ADDR0_EN / ADDR1_EN, TGT_EN = 0, EN = 0 (4.5.1)
// -----------------------------------------------------------------------------
class i2c_tgt_addr_filter_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_addr_filter_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_addr_filter_vseq s = i2c_tgt_addr_filter_vseq::type_id::create("af");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_tgt_addr_filter_test
