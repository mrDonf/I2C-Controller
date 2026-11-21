// -----------------------------------------------------------------------------
// i2c_arb_hold_test  [VP-BUS-17]
//   corner: SCL pulled low by another device inside the START / Sr hold (clock synchronization, no loss), before
//   the START is visible and after the SDA release of the STOP (arbitration lost) (4.4.3, 4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_hold_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_hold_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_hold_vseq     s = i2c_arb_hold_vseq::type_id::create("hold");
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    if (!s.randomize() with { kind == AH_START_SYNC; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == AH_RSTART_SYNC; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == AH_START_SCL_LOW; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == AH_STOP_WAIT; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    repeat (2) rand_start(s);                // random kind and pulse position
    rand_start(x);
  endtask
endclass : i2c_arb_hold_test
