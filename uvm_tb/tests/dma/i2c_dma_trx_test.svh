// -----------------------------------------------------------------------------
// i2c_dma_trx_test  [VP-DMA-03]
//   DMA SEL = TRX with peripheral flow control: one operation = one transaction (6.15.3)
// -----------------------------------------------------------------------------
class i2c_dma_trx_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_trx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 1;
  endfunction

  virtual task run_scenario();
    i2c_dma_trx_vseq s = i2c_dma_trx_vseq::type_id::create("trx");
    start_seq(s);
  endtask
endclass : i2c_dma_trx_test
