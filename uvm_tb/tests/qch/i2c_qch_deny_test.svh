// -----------------------------------------------------------------------------
// i2c_qch_deny_test  [VP-QCH-02]
// -----------------------------------------------------------------------------
class i2c_qch_deny_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_deny_test)
  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  virtual task run_scenario();
    i2c_qch_deny_vseq     dn = i2c_qch_deny_vseq::type_id::create("deny");
    i2c_ip_qch_cycle_vseq cy = i2c_ip_qch_cycle_vseq::type_id::create("cy");
    start_seq(dn);
    rand_start(cy);                                         // the IP is idle again: accepted
  endtask
endclass : i2c_qch_deny_test
