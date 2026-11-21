// -----------------------------------------------------------------------------
// i2c_rand_ctrl_test  [VP-RND-02]
//   random controller-only traffic, long transfers (up to 3 x FIFO_DEPTH)
// -----------------------------------------------------------------------------
class i2c_rand_ctrl_test extends i2c_rand_test;
  `uvm_component_utils(i2c_rand_ctrl_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_rand_traffic_vseq s = i2c_rand_traffic_vseq::type_id::create("rnd");
    if (!s.randomize() with { n_iter == local::n_iter; w_ewr == 0; w_erd == 0; w_ewrrd == 0; max_len == 3 * local::depth; })
      `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_rand_ctrl_test
