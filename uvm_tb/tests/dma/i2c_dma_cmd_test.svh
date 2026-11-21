// -----------------------------------------------------------------------------
// i2c_dma_cmd_test  [VP-DMA-01]
//   DMA SEL = CMD with DMAC flow control: BLOCK / LAST_OKAY, DMA_DONE on irq_o (6.15.1)
// -----------------------------------------------------------------------------
class i2c_dma_cmd_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_cmd_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_cmd_vseq s = i2c_dma_cmd_vseq::type_id::create("cmd");
    start_seq(s);
  endtask
endclass : i2c_dma_cmd_test
