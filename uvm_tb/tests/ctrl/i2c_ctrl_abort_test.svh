// -----------------------------------------------------------------------------
// i2c_ctrl_abort_test  [VP-CTL-12]
//   CTRL.ABORT idle / waiting for the bus / stalled write / stalled read / read address / read byte in flight,
//   random order (4.4.9, 6.13)
// -----------------------------------------------------------------------------
class i2c_ctrl_abort_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_abort_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_abort_vseq   ab = i2c_ctrl_abort_vseq::type_id::create("ab");
    i2c_ip_ctrl_xfer_vseq x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    i2c_ip_abort_e kinds[$] = '{AB_IDLE, AB_WRITE_STALL, AB_READ_STALL, AB_READ_INFLIGHT, AB_WAITBUS, AB_READ_ADDR};
    kinds.shuffle();
    foreach (kinds[i]) begin
      if (!ab.randomize() with { kind == local::kinds[i]; }) `uvm_fatal("TEST", "randomize")
      start_seq(ab);
      rand_start(x);                       // normal operation after every abort
    end
  endtask
endclass : i2c_ctrl_abort_test
