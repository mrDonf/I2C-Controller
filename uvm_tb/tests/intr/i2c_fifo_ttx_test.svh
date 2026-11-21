// -----------------------------------------------------------------------------
// i2c_fifo_ttx_test  [VP-FIFO-02]
//   TTX FIFO: level, threshold, full, overflow PSLVERR + FIFO_ERR, flush (4.7)
// -----------------------------------------------------------------------------
class i2c_fifo_ttx_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_fifo_ttx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_fifo_ttx_vseq s = i2c_fifo_ttx_vseq::type_id::create("ttx");
    start_seq(s);
  endtask
endclass : i2c_fifo_ttx_test
