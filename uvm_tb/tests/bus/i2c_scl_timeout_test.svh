// -----------------------------------------------------------------------------
// i2c_scl_timeout_test  [VP-BUS-06]
//   SCL low timeout: controller abort without STOP, target stretch released, TOUT_EN = 0 (4.3.5, 4.4.9, 4.5.9)
// -----------------------------------------------------------------------------
class i2c_scl_timeout_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_scl_timeout_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_timeout_cfg_vseq tc  = i2c_ip_timeout_cfg_vseq::type_id::create("tc");
    i2c_tout_ctrl_vseq      tct = i2c_tout_ctrl_vseq::type_id::create("tct");
    i2c_tout_tgt_vseq       ttg = i2c_tout_tgt_vseq::type_id::create("ttg");
    i2c_tout_off_vseq       off = i2c_tout_off_vseq::type_id::create("off");
    i2c_ip_ctrl_xfer_vseq   x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    if (!tc.randomize() with { tout_en == 1; tout_val inside {[1500:3000]}; }) `uvm_fatal("TEST", "randomize")
    start_seq(tc);                         // 15 .. 30 us
    repeat (2) rand_start(tct);
    start_seq(ttg);
    if (!tc.randomize() with { tout_en == 0; tout_val == 2000; }) `uvm_fatal("TEST", "randomize")
    start_seq(tc);
    start_seq(off);
    rand_start(x);
  endtask
endclass : i2c_scl_timeout_test
