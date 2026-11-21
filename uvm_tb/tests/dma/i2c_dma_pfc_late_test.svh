// -----------------------------------------------------------------------------
// i2c_dma_pfc_late_test  [VP-DMA-11]
//   corner: RX peripheral flow control with the LAST entry already in CRX (lpos > BLK, = BLK, < BLK), 4.9.2
// -----------------------------------------------------------------------------
class i2c_dma_pfc_late_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_pfc_late_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_pfc_late_vseq s = i2c_dma_pfc_late_vseq::type_id::create("late");
    repeat (6) rand_start(s);
  endtask
endclass : i2c_dma_pfc_late_test
