// -----------------------------------------------------------------------------
// i2c_dma_ttx_test  [VP-DMA-04]
//   DMA SEL = TTX: the DMAC loads the bytes read by the external controller (6.15.4)
// -----------------------------------------------------------------------------
class i2c_dma_ttx_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_ttx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 1;
  endfunction

  virtual task run_scenario();
    i2c_dma_ttx_vseq s = i2c_dma_ttx_vseq::type_id::create("ttx");
    start_seq(s);
    repeat (2) begin
      if (!s.randomize() with { n inside {[1:16]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(s);
    end
  endtask
endclass : i2c_dma_ttx_test
