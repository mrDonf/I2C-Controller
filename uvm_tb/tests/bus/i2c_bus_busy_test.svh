// -----------------------------------------------------------------------------
// i2c_bus_busy_test  [VP-BUS-10]
//   bus busy / free detection: IDLE_VAL after enable, S..P, tBUF before bus_free and before the DUT START (4.3.3)
// -----------------------------------------------------------------------------
class i2c_bus_busy_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_bus_busy_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 0;
  endfunction

  virtual task run_scenario();
    i2c_bus_busy_vseq s = i2c_bus_busy_vseq::type_id::create("bb");
    start_seq(s);
  endtask
endclass : i2c_bus_busy_test
