// -----------------------------------------------------------------------------
// i2c_rand_tgt_test  [VP-RND-03]
//   random target-only traffic (external controller writes / reads / write-reads)
// -----------------------------------------------------------------------------
class i2c_rand_tgt_test extends i2c_rand_test;
  `uvm_component_utils(i2c_rand_tgt_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_rand_traffic_vseq s = i2c_rand_traffic_vseq::type_id::create("rnd");
    if (!s.randomize() with { n_iter == local::n_iter; w_cwr == 0; w_crd == 0; w_cwrrd == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_rand_tgt_test
