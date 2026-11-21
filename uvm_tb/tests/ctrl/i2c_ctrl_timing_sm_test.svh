// -----------------------------------------------------------------------------
// i2c_ctrl_timing_sm_test  [VP-CTL-05]
//   Standard-mode timing (100 kHz)
// -----------------------------------------------------------------------------
class i2c_ctrl_timing_sm_test extends i2c_ctrl_timing_test;
  `uvm_component_utils(i2c_ctrl_timing_sm_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_speed = SPD_SM;
  endfunction
endclass : i2c_ctrl_timing_sm_test
