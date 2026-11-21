// -----------------------------------------------------------------------------
// i2c_tgt_stretch_test  [VP-TGT-08]
//   byte-level clock stretching while TRX lacks room, no loss, no TRX_OVF (4.5.4, 8.4)
// -----------------------------------------------------------------------------
class i2c_tgt_stretch_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_stretch_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_stretch_vseq s = i2c_tgt_stretch_vseq::type_id::create("st");
    rand_start(s);
  endtask
endclass : i2c_tgt_stretch_test
