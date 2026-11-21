// -----------------------------------------------------------------------------
// i2c_ctrl_nakok_test  [VP-CTL-10]
//   NAKOK: refused data / absent address do not abort the transfer, no NACK event (4.4.4)
// -----------------------------------------------------------------------------
class i2c_ctrl_nakok_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_nakok_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ext_dev_cfg_vseq dev = i2c_ip_ext_dev_cfg_vseq::type_id::create("dev");
    i2c_ctrl_nakok_vseq     nk  = i2c_ctrl_nakok_vseq::type_id::create("nk");
    dev.slv = 1;
    if (!dev.randomize() with { nack_after == 2; stretch_ns == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(dev);
    nk.addr_case = 0;
    start_seq(nk);
    if (!dev.randomize() with { nack_after == -1; stretch_ns == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(dev);
    nk.addr_case = 1;
    start_seq(nk);
  endtask
endclass : i2c_ctrl_nakok_test
