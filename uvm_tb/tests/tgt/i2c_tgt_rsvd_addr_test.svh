// -----------------------------------------------------------------------------
// i2c_tgt_rsvd_addr_test  [VP-TGT-12]
//   reserved addresses / CBUS / 1111_1xx never acknowledged, Hs controller codes -> HS_DET (4.5.1, 4.3.4)
// -----------------------------------------------------------------------------
class i2c_tgt_rsvd_addr_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_rsvd_addr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_rsvd_addr_vseq s = i2c_tgt_rsvd_addr_vseq::type_id::create("rsvd");
    start_seq(s);
  endtask
endclass : i2c_tgt_rsvd_addr_test
