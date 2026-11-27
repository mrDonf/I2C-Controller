// -----------------------------------------------------------------------------
// i2c_qch_reset_test  [VP-QCH-07]
//   reset state of the channel (QREQN_RST build parameter), first access, PRESETn while stopped / running.
//   The clock controller does not leave Q_STOPPED by itself after reset: the first APB access must wake the IP.
// -----------------------------------------------------------------------------
class i2c_qch_reset_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_reset_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    do_init = 0;
  endfunction

  virtual function void configure_env();
    super.configure_env();
    cfg.qch_cfg.exit_after_reset = 0;
  endfunction

  virtual task run_scenario();
    i2c_qch_reset_vseq    rs = i2c_qch_reset_vseq::type_id::create("rs");
    i2c_ip_qch_cycle_vseq cy = i2c_ip_qch_cycle_vseq::type_id::create("cy");
    start_seq(rs);
    rand_start(cy);
  endtask
endclass : i2c_qch_reset_test
