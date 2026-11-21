// -----------------------------------------------------------------------------
// i2c_clk_sync_test  [VP-BUS-04]
//   another device stretches SCL inside bytes at random bits (clock synchronisation, F6 / F7)
// -----------------------------------------------------------------------------
class i2c_clk_sync_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_clk_sync_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_clk_sync_vseq s = i2c_clk_sync_vseq::type_id::create("cs");
    start_seq(s);
    repeat (3) rand_start(s);
  endtask
endclass : i2c_clk_sync_test
