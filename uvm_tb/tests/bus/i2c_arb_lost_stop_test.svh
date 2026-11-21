// -----------------------------------------------------------------------------
// i2c_arb_lost_stop_test  [VP-BUS-15]
//   corner: SDA held low after the STOP release: STOP watchdog = arbitration lost, no hang (4.4.3, 4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_lost_stop_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_stop_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_stop_vseq s = i2c_arb_lost_stop_vseq::type_id::create("arbstop");
    i2c_ip_ctrl_xfer_vseq  x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    repeat (2) rand_start(s);
    rand_start(x);
  endtask
endclass : i2c_arb_lost_stop_test
