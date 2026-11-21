// -----------------------------------------------------------------------------
// i2c_ctrl_back2back_test  [VP-CTL-18]
//   corner: several complete transactions queued in the CMD FIFO at once (STOP / tBUF / START sequencing)
// -----------------------------------------------------------------------------
class i2c_ctrl_back2back_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_back2back_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_back2back_vseq bb2 = i2c_ctrl_back2back_vseq::type_id::create("bb2");
    repeat (3) begin
      if (!bb2.randomize() with { 3 * n_txn <= 2 * local::depth; }) `uvm_fatal("TEST", "randomize")
      start_seq(bb2);
    end
  endtask
endclass : i2c_ctrl_back2back_test
