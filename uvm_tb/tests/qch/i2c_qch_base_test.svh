// -----------------------------------------------------------------------------
// i2c_qch_base_test - group base of the Q-Channel tests (VP-QCH, spec 3.8, 4.12, 6.17)
//   controller + target enabled, APB requester timeout raised (an access from Q_STOPPED waits for the wake-up),
//   automatic wake-up of the clock-controller model with a random latency
// -----------------------------------------------------------------------------
class i2c_qch_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_qch_base_test)

  int unsigned wake_lat_max = 8;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 1; init_tgt = 1;
  endfunction

  virtual function void configure_env();
    cfg.apb_cfg.timeout_cycles   = 400;
    cfg.qch_cfg.auto_wake        = 1;
    cfg.qch_cfg.wake_lat_min     = 0;
    cfg.qch_cfg.wake_lat_max     = wake_lat_max;
  endfunction
endclass : i2c_qch_base_test
