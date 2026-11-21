// -----------------------------------------------------------------------------
// i2c_intr_rand_test  [VP-INT-02]
//   random event order, several events and random INTR_EN at once (4.8)
// -----------------------------------------------------------------------------
class i2c_intr_rand_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_intr_rand_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_intr_event_vseq ev = i2c_intr_event_vseq::type_id::create("ev");
    i2c_intr_multi_vseq mu = i2c_intr_multi_vseq::type_id::create("mu");
    ev.shuffle = 1;
    start_seq(ev);
    mu.n_iter = plusarg_int("N_ITER", 40);
    start_seq(mu);
  endtask
endclass : i2c_intr_rand_test
