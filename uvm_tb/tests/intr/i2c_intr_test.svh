// -----------------------------------------------------------------------------
// i2c_intr_test  [VP-INT-01/03]
//   event bits: INTR_TEST, W1C (lanes), INTR_EN routing to the 4 lines; level bits (4.8)
// -----------------------------------------------------------------------------
class i2c_intr_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_intr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_intr_event_vseq ev  = i2c_intr_event_vseq::type_id::create("ev");
    i2c_intr_level_vseq lvl = i2c_intr_level_vseq::type_id::create("lvl");
    start_seq(ev);
    start_seq(lvl);
  endtask
endclass : i2c_intr_test
