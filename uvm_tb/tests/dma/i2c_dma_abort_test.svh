// -----------------------------------------------------------------------------
// i2c_dma_abort_test  [VP-DMA-09]
//   controller abort (NACK) with SEL = CMD clears DMA_CTRL.EN, recovery (4.4.9, 6.13)
// -----------------------------------------------------------------------------
class i2c_dma_abort_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_abort_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_abort_vseq s = i2c_dma_abort_vseq::type_id::create("abort");
    start_seq(s);
  endtask
endclass : i2c_dma_abort_test
