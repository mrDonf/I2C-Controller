// -----------------------------------------------------------------------------
// i2c_arb_lost_sp_test  [VP-BUS-16]
//   corner: SCL pulled low by another device during the Sr / STOP setup: arbitration lost, recovery (4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_lost_sp_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_sp_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_sp_vseq  s = i2c_arb_lost_sp_vseq::type_id::create("arbsp");
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    if (!s.randomize() with { kind == SP_RSTART; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == SP_STOP; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    rand_start(x);
  endtask
endclass : i2c_arb_lost_sp_test
