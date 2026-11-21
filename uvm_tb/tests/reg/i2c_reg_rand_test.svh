// -----------------------------------------------------------------------------
// i2c_reg_rand_test  [VP-REG-10]
//   random register accesses: values, strobes, PADDR[1:0], RO / unmapped offsets, mirror checks (+N_ITER)
// -----------------------------------------------------------------------------
class i2c_reg_rand_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_rand_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  int unsigned n;
  function void start_of_simulation_phase(uvm_phase phase); n = plusarg_int("N_ITER", 400); endfunction

  virtual task run_scenario();
    i2c_rand_reg_vseq s = i2c_rand_reg_vseq::type_id::create("rreg");
    if (!s.randomize() with { n_iter == local::n; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_reg_rand_test
