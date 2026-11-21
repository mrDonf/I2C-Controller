// -----------------------------------------------------------------------------
// i2c_tgt_devid_corner_test  [VP-TGT-18]
//   corner: Device ID combined formats D1..D11: 0xF8 without address byte, repeated 0xF8, 0xF9 not armed / after a
//   normal frame / DEVID_EN latched 0 at the Sr, repeated 0xF9, reserved ADDR0, no TRX entries (4.5.6)
// -----------------------------------------------------------------------------
class i2c_tgt_devid_corner_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_devid_corner_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_tgt_devid_corner_vseq s = i2c_tgt_devid_corner_vseq::type_id::create("devc");
    i2c_tgt_devid_vseq        d = i2c_tgt_devid_vseq::type_id::create("devid");
    rand_start(s);
    rand_start(d);                           // the regular sequence still works afterwards
    rand_start(s);
  endtask
endclass : i2c_tgt_devid_corner_test
