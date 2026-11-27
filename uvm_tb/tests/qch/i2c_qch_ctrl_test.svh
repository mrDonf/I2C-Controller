// -----------------------------------------------------------------------------
// i2c_qch_ctrl_test  [VP-QCH-05]
// -----------------------------------------------------------------------------
class i2c_qch_ctrl_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_ctrl_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_tgt = 0;
  endfunction

  virtual task run_scenario();
    i2c_qch_ctrl_vseq     cx = i2c_qch_ctrl_vseq::type_id::create("cx");
    i2c_ip_qch_cycle_vseq cy = i2c_ip_qch_cycle_vseq::type_id::create("cy");
    rand_start(cx);
    rand_start(cy);
  endtask
endclass : i2c_qch_ctrl_test
