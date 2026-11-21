// -----------------------------------------------------------------------------
// i2c_dma_blk_test  [VP-DMA-05]
//   DMA_BLK = 0 (SINGLE only, reserved ack = OKAY), DMA_BLK > FIFO_DEPTH clamped (5.2.23, 4.9.3)
// -----------------------------------------------------------------------------
class i2c_dma_blk_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_blk_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_blk_vseq s = i2c_dma_blk_vseq::type_id::create("blk");
    start_seq(s);
  endtask
endclass : i2c_dma_blk_test
