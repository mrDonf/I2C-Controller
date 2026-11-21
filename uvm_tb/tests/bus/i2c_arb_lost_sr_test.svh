// -----------------------------------------------------------------------------
// i2c_arb_lost_sr_test  [VP-BUS-13]
//   corner: arbitration lost while sending a repeated START (SDA low when SCL rises, 4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_lost_sr_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_sr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_sr_vseq s = i2c_arb_lost_sr_vseq::type_id::create("arbsr");
    repeat (2) rand_start(s);
  endtask
endclass : i2c_arb_lost_sr_test
