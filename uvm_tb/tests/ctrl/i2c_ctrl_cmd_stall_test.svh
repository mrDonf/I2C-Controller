// -----------------------------------------------------------------------------
// i2c_ctrl_cmd_stall_test  [VP-CTL-06]
//   transfer longer than the CMD FIFO: starvation (CMD_STALL, SCL held), CMD_THR-driven refill (6.3, 4.4.3)
// -----------------------------------------------------------------------------
class i2c_ctrl_cmd_stall_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_cmd_stall_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_cmd_stall_vseq st = i2c_ctrl_cmd_stall_vseq::type_id::create("st");
    start_seq(st);                         // default: 3 x FIFO_DEPTH + 3 bytes, CMD_THR 4
    if (!st.randomize() with { n inside {[local::depth + 1:4 * local::depth]}; thr < local::depth; }) `uvm_fatal("TEST", "randomize")
    start_seq(st);
  endtask
endclass : i2c_ctrl_cmd_stall_test
