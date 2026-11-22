// -----------------------------------------------------------------------------
// i2c_dma_clr_test  [VP-DMA-13]
//   DMAC without DENY: request withdrawn by DMA_CTRL.CLR (EN = 0 / EN = 1, IDLE), late ack ignored, ack wins over
//   CLR in the same cycle (4.9.3, 4.9.7)
// -----------------------------------------------------------------------------
class i2c_dma_clr_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_clr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_clr_vseq      s = i2c_dma_clr_vseq::type_id::create("clr");
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(s);
    rand_start(x);
  endtask
endclass : i2c_dma_clr_test
