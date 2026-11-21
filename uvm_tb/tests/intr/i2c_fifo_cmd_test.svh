// -----------------------------------------------------------------------------
// i2c_fifo_cmd_test  [VP-FIFO-01]
//   CMD FIFO: level, threshold, full, overflow PSLVERR + FIFO_ERR, clamp, flush, kept with EN = 0 (4.7)
// -----------------------------------------------------------------------------
class i2c_fifo_cmd_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_fifo_cmd_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_fifo_cmd_vseq s = i2c_fifo_cmd_vseq::type_id::create("cmd");
    start_seq(s);
  endtask
endclass : i2c_fifo_cmd_test
