// -----------------------------------------------------------------------------
// i2c_dual_role_test  [VP-SYS-01]
//   controller and target of the same IP sharing the bus with an external controller (4.6)
// -----------------------------------------------------------------------------
class i2c_dual_role_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_dual_role_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dual_role_vseq s = i2c_dual_role_vseq::type_id::create("dual");
    if (!s.randomize() with { n_iter inside {[4:8]}; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_dual_role_test
