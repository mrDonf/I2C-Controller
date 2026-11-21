// -----------------------------------------------------------------------------
// i2c_dma_rand_test  [VP-DMA-10]
//   random DMA operations: SEL, PFC, BLK, lengths (+N_ITER)
// -----------------------------------------------------------------------------
class i2c_dma_rand_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_rand_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 1; init_tgt = 1;
  endfunction

  int unsigned n;
  function void start_of_simulation_phase(uvm_phase phase); n = plusarg_int("N_ITER", 16); endfunction

  virtual task run_scenario();
    i2c_dma_rand_vseq s = i2c_dma_rand_vseq::type_id::create("rdma");
    if (!s.randomize() with { n_iter == local::n; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_dma_rand_test
