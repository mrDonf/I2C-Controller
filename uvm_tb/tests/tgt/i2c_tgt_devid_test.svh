// -----------------------------------------------------------------------------
// i2c_tgt_devid_test  [VP-TGT-14]
//   Device ID: 0xF8 / 0xF9 sequence, wrap-around, not armed / foreign / disabled, Sr disarms (4.5.6, 6.9)
// -----------------------------------------------------------------------------
class i2c_tgt_devid_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_devid_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_devid_vseq s = i2c_tgt_devid_vseq::type_id::create("devid");
    repeat (2) rand_start(s);
  endtask
endclass : i2c_tgt_devid_test
