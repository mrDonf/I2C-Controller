// -----------------------------------------------------------------------------
// i2c_arb_lost_data_test  [VP-BUS-03]
//   arbitration lost in a data byte (same address): no CMPL, CSTAT.ARB_LOST, data of the winner (4.4.6)
// -----------------------------------------------------------------------------
class i2c_arb_lost_data_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_data_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_data_vseq s = i2c_arb_lost_data_vseq::type_id::create("arbd");
    repeat (3) rand_start(s);
  endtask
endclass : i2c_arb_lost_data_test
