// -----------------------------------------------------------------------------
// i2c_ctrl_crx_full_test  [VP-CTL-07]
//   read longer than CRX with slow software and a stretching target: controller holds SCL, no overflow (4.4.3, F7)
// -----------------------------------------------------------------------------
class i2c_ctrl_crx_full_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_crx_full_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ext_dev_cfg_vseq dev = i2c_ip_ext_dev_cfg_vseq::type_id::create("dev");
    i2c_ctrl_crx_full_vseq  rd  = i2c_ctrl_crx_full_vseq::type_id::create("rd");
    dev.slv = 0;
    if (!dev.randomize() with { stretch_ns == 3000; }) `uvm_fatal("TEST", "randomize")
    start_seq(dev);                        // the external target stretches after every ACK
    start_seq(rd);
    if (!dev.randomize() with { stretch_ns == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(dev);
    if (!rd.randomize() with { nrd inside {[local::depth + 1:3 * local::depth]}; }) `uvm_fatal("TEST", "randomize")
    start_seq(rd);
  endtask
endclass : i2c_ctrl_crx_full_test
