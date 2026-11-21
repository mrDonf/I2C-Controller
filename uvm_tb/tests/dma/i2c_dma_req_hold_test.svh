// -----------------------------------------------------------------------------
// i2c_dma_req_hold_test  [VP-DMA-06]
//   a request is never withdrawn (EN = 0, SWRST), SEL / PFC / BLK locked while pending (4.9.2, 5.2.1)
// -----------------------------------------------------------------------------
class i2c_dma_req_hold_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_req_hold_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_req_hold_vseq s = i2c_dma_req_hold_vseq::type_id::create("hold");
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(s);
    rand_start(x);
  endtask
endclass : i2c_dma_req_hold_test
