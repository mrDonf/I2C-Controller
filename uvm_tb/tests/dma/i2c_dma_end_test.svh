// -----------------------------------------------------------------------------
// i2c_dma_end_test  [VP-DMA-12]
//   end handshake from the DMAC (dma_end_req_i / dma_end_ack_o, DMA-350 trigger output): ack one cycle later,
//   DMA_STAT.END, END_EN = 0 vs 1, pulse source, request kept at the end event, END_EN lock (4.9.6, 5.2.22,
//   5.2.24)
// -----------------------------------------------------------------------------
class i2c_dma_end_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_dma_end_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_dma_end_vseq      s = i2c_dma_end_vseq::type_id::create("end");
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(s);
    rand_start(x);
  endtask
endclass : i2c_dma_end_test
