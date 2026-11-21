// -----------------------------------------------------------------------------
// i2c_ctrl_timing_fm_test  [VP-CTL-05]
//   Fast-mode timing (400 kHz)
// -----------------------------------------------------------------------------
class i2c_ctrl_timing_fm_test extends i2c_ctrl_timing_test;
  `uvm_component_utils(i2c_ctrl_timing_fm_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_speed = SPD_FM;
  endfunction
endclass : i2c_ctrl_timing_fm_test
