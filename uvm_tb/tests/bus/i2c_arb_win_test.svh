// -----------------------------------------------------------------------------
// i2c_arb_win_test  [VP-BUS-02]
//   the other controller loses against the DUT; clock synchronisation during the address (4.4.6, 4.4.2)
// -----------------------------------------------------------------------------
class i2c_arb_win_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_win_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_arb_win_vseq s = i2c_arb_win_vseq::type_id::create("win");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_arb_win_test
