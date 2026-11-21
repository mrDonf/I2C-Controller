// -----------------------------------------------------------------------------
// i2c_ctrl_data_nack_test  [VP-CTL-09]
//   data NACK at every byte position: NACK_DATA, STOP, CMPL, CMD flushed (4.4.4)
// -----------------------------------------------------------------------------
class i2c_ctrl_data_nack_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_data_nack_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ext_dev_cfg_vseq dev = i2c_ip_ext_dev_cfg_vseq::type_id::create("dev");
    i2c_ctrl_data_nack_vseq dn  = i2c_ctrl_data_nack_vseq::type_id::create("dn");
    i2c_ip_ctrl_xfer_vseq   x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    dev.slv = 1;
    for (int k = 0; k < 4; k++) begin
      if (!dev.randomize() with { nack_after == k; stretch_ns == 0; }) `uvm_fatal("TEST", "randomize")
      start_seq(dev);
      dn.nack_after = k;
      start_seq(dn);
    end
    if (!dev.randomize() with { nack_after == -1; stretch_ns == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(dev);
    rand_start(x);
  endtask
endclass : i2c_ctrl_data_nack_test
