// -----------------------------------------------------------------------------
// i2c_busmon_test  [VP-BUS-09]
//   BUSMON: SCL_F / SDA_F, line override with EN = 0 (bit-bang recovery), ignored with EN = 1 (4.10)
// -----------------------------------------------------------------------------
class i2c_busmon_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_busmon_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 0;
  endfunction

  virtual task run_scenario();
    i2c_busmon_vseq s = i2c_busmon_vseq::type_id::create("bm");
    start_seq(s);
  endtask
endclass : i2c_busmon_test
