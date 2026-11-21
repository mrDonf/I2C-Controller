// -----------------------------------------------------------------------------
// i2c_ctrl_timing_test  [VP-CTL-05]
//   bus timing of the controller vs 4.4.2 / 6.2 / UM10204, measured while traffic runs (base of the speed tests)
// -----------------------------------------------------------------------------
class i2c_ctrl_timing_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_timing_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_timing_meas_vseq  meas = i2c_ip_timing_meas_vseq::type_id::create("meas");
    i2c_ctrl_timing_chk_vseq tchk = i2c_ctrl_timing_chk_vseq::type_id::create("tchk");
    i2c_ip_ctrl_xfer_vseq    x    = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    fork
      start_seq(meas);                     // runs in parallel with the traffic
      begin
        if (!x.randomize() with { kind == CX_WRITE; data.size() inside {[3:6]}; }) `uvm_fatal("TEST", "randomize")
        start_seq(x);
        if (!x.randomize() with { kind == CX_WRITE_READ; rd_len inside {[2:4]}; }) `uvm_fatal("TEST", "randomize")
        start_seq(x);
        if (!x.randomize() with { kind == CX_WRITE; }) `uvm_fatal("TEST", "randomize")
        start_seq(x);
        meas.stop_req = 1;
      end
    join
    tchk.meas = meas;
    start_seq(tchk);
  endtask
endclass : i2c_ctrl_timing_test
