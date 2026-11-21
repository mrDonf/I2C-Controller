// -----------------------------------------------------------------------------
// i2c_ctrl_write_test  [VP-CTL-01]
//   controller write (6.3): random targets / pointers / lengths, address-only write, idle state afterwards
// -----------------------------------------------------------------------------
class i2c_ctrl_write_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_write_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ctrl_xfer_vseq  wr   = i2c_ip_ctrl_xfer_vseq::type_id::create("wr");
    i2c_ctrl_idle_chk_vseq idle = i2c_ctrl_idle_chk_vseq::type_id::create("idle");
    repeat (8) begin
      if (!wr.randomize() with { kind == CX_WRITE; data.size() inside {[1:2 * local::depth]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(wr);
    end
    if (!wr.randomize() with { kind == CX_WRITE; no_ptr == 1; data.size() == 0; }) `uvm_fatal("TEST", "randomize")
    start_seq(wr);                         // S A W P
    start_seq(idle);
  endtask
endclass : i2c_ctrl_write_test
