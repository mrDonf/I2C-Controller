// -----------------------------------------------------------------------------
// i2c_qch_handshake_test  [VP-QCH-01]
//   accept / stop / exit handshakes of an idle IP (CTRL.EN 0 and 1, clock gated and running), register access after
//   every exit, then a normal transfer
// -----------------------------------------------------------------------------
class i2c_qch_handshake_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_handshake_test)
  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  virtual task run_scenario();
    i2c_qch_handshake_vseq hs  = i2c_qch_handshake_vseq::type_id::create("hs");
    i2c_ip_qch_cycle_vseq  cyc = i2c_ip_qch_cycle_vseq::type_id::create("cyc");
    i2c_ip_ctrl_xfer_vseq  x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    rand_start(hs);
    if (!cyc.randomize() with { hold_ns inside {[20000:60000]}; gate_clk == 1; }) `uvm_fatal("TEST", "cyc")
    start_seq(cyc);                                         // long stop with the IP enabled and the bus idle
    if (!x.randomize() with { kind == CX_WRITE_READ; }) `uvm_fatal("TEST", "x")
    start_seq(x);
  endtask
endclass : i2c_qch_handshake_test
