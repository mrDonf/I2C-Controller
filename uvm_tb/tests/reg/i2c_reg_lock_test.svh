// -----------------------------------------------------------------------------
// i2c_reg_lock_test  [VP-REG-05]
//   timing / FILTER / BUSIDLE locked while CTRL.EN = 1, the running configuration is not disturbed (3.7, 5.1)
// -----------------------------------------------------------------------------
class i2c_reg_lock_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_reg_lock_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_reg_lock_vseq     lock = i2c_reg_lock_vseq::type_id::create("lock");
    i2c_ip_ctrl_xfer_vseq xfer = i2c_ip_ctrl_xfer_vseq::type_id::create("xfer");
    rand_start(xfer);
    start_seq(lock);                       // with the IP enabled and configured
    rand_start(xfer);                      // still running with the programmed timing
  endtask
endclass : i2c_reg_lock_test
