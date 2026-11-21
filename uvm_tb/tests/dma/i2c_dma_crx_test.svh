// -----------------------------------------------------------------------------
// i2c_dma_crx_test  [VP-DMA-02]
//   DMA SEL = CRX: DMAC flow control with DENY of SINGLE, peripheral flow control LAST_BLOCK (6.15.2, 4.9.4)
// -----------------------------------------------------------------------------
class i2c_dma_crx_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_crx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_crx_vseq s = i2c_dma_crx_vseq::type_id::create("crx");
    if (!s.randomize() with { pfc == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { pfc == 1; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_dma_crx_test
