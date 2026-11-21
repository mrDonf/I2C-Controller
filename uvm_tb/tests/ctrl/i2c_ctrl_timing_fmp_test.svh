// -----------------------------------------------------------------------------
// i2c_ctrl_timing_fmp_test  [VP-CTL-05]
//   Fast-mode Plus timing (1 MHz)
// -----------------------------------------------------------------------------
class i2c_ctrl_timing_fmp_test extends i2c_ctrl_timing_test;
  `uvm_component_utils(i2c_ctrl_timing_fmp_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_speed = SPD_FMP;
  endfunction
endclass : i2c_ctrl_timing_fmp_test
