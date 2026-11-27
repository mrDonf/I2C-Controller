// -----------------------------------------------------------------------------
// i2c_qch_dma_test  [VP-QCH-06]
// -----------------------------------------------------------------------------
class i2c_qch_dma_test extends i2c_dma_base_test;
  `uvm_component_utils(i2c_qch_dma_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 1;
  endfunction

  virtual function void configure_env();
    cfg.apb_cfg.timeout_cycles = 400;
    cfg.qch_cfg.wake_lat_max   = 8;
  endfunction

  virtual task run_scenario();
    i2c_qch_dma_vseq      dm = i2c_qch_dma_vseq::type_id::create("dm");
    i2c_ip_qch_cycle_vseq cy = i2c_ip_qch_cycle_vseq::type_id::create("cy");
    rand_start(dm);
    rand_start(cy);
  endtask
endclass : i2c_qch_dma_test
