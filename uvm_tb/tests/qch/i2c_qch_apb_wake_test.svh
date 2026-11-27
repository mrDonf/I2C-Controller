// -----------------------------------------------------------------------------
// i2c_qch_apb_wake_test  [VP-QCH-03]
//   APB accesses from Q_STOPPED, first with an immediate wake-up, then with a slow clock controller
// -----------------------------------------------------------------------------
class i2c_qch_apb_wake_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_apb_wake_test)
  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  virtual task run_scenario();
    i2c_qch_apb_wake_vseq fast = i2c_qch_apb_wake_vseq::type_id::create("fast");
    i2c_qch_apb_wake_vseq slow = i2c_qch_apb_wake_vseq::type_id::create("slow");
    cfg.qch_cfg.wake_lat_min = 0; cfg.qch_cfg.wake_lat_max = 0;
    rand_start(fast);
    cfg.qch_cfg.wake_lat_min = 20; cfg.qch_cfg.wake_lat_max = 120;
    rand_start(slow);
  endtask
endclass : i2c_qch_apb_wake_test
