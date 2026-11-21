// -----------------------------------------------------------------------------
// i2c_tgt_gcall_test  [VP-TGT-11]
//   general call: GC_EN, GC_SWRST, TSTAT.GC, Sr to the own address after a general call (4.5.1)
// -----------------------------------------------------------------------------
class i2c_tgt_gcall_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_gcall_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_gcall_vseq s = i2c_tgt_gcall_vseq::type_id::create("gc");
    start_seq(s);
  endtask
endclass : i2c_tgt_gcall_test
