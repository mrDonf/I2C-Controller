// -----------------------------------------------------------------------------
// i2c_dma_ack_type_test  [VP-DMA-08]
//   ack types: LAST_* answered LAST_OKAY, reserved ack 2'b11 = OKAY (4.9.3)
// -----------------------------------------------------------------------------
class i2c_dma_ack_type_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_ack_type_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_ack_type_vseq s = i2c_dma_ack_type_vseq::type_id::create("ack");
    start_seq(s);
  endtask
endclass : i2c_dma_ack_type_test
