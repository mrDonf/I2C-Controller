// -----------------------------------------------------------------------------
// i2c_bus_err_test  [VP-BUS-07]
//   misplaced START / STOP at random bits: target back to idle, controller abort without STOP (4.3.2, 4.4.9,
//   4.5.9)
// -----------------------------------------------------------------------------
class i2c_bus_err_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_bus_err_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_bus_err_tgt_vseq  et = i2c_bus_err_tgt_vseq::type_id::create("et");
    i2c_bus_err_ctrl_vseq ec = i2c_bus_err_ctrl_vseq::type_id::create("ec");
    i2c_ip_ctrl_xfer_vseq x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    i2c_ip_ext_xfer_vseq  e  = i2c_ip_ext_xfer_vseq::type_id::create("e");
    repeat (2) begin
      rand_start(et);
      rand_start(ec);
    end
    rand_start(x);
    if (!e.randomize() with { kind == EX_WRITE; dst == DST_OWN0; data.size() <= local::depth - 2; }) `uvm_fatal("TEST", "randomize")
    e.exp_ack = 1;
    start_seq(e);                          // the target works after the bus errors
    begin i2c_ip_trx_collect_vseq c = i2c_ip_trx_collect_vseq::type_id::create("c"); c.stop_req = 1; start_seq(c); end
  endtask
endclass : i2c_bus_err_test
