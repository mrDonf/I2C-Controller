// -----------------------------------------------------------------------------
// i2c_dma_deny_test  [VP-DMA-07]
//   DENY: no new request until the level changes; LAST_SINGLE / LAST_BLOCK of a stopped channel (4.9.2, 4.9.5)
// -----------------------------------------------------------------------------
class i2c_dma_deny_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_deny_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_deny_vseq s = i2c_dma_deny_vseq::type_id::create("deny");
    start_seq(s);
  endtask
endclass : i2c_dma_deny_test
