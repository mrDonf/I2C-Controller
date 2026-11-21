// -----------------------------------------------------------------------------
// i2c_arb_lost_nack_test  [VP-BUS-14]
//   corner: two controllers read the same target, the DUT NACK loses against the other ACK (4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_lost_nack_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_nack_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_nack_vseq s = i2c_arb_lost_nack_vseq::type_id::create("arbnack");
    i2c_ip_ctrl_xfer_vseq  x = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(s);
    rand_start(x);
  endtask
endclass : i2c_arb_lost_nack_test
