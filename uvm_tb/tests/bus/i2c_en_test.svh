// -----------------------------------------------------------------------------
// i2c_en_test  [VP-BUS-12]
//   EN -> 0 during a controller transfer (bus released, FIFO kept); TGT_EN -> 0 inside a transaction (4.1, 4.5.9)
// -----------------------------------------------------------------------------
class i2c_en_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_en_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_en_ctrl_vseq      ec = i2c_en_ctrl_vseq::type_id::create("ec");
    i2c_tgt_en_vseq       et = i2c_tgt_en_vseq::type_id::create("et");
    i2c_ip_ctrl_xfer_vseq x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(ec);
    rand_start(x);
    init_ctrl = 0; init_tgt = 1;
    configure_dut();                       // re-initialise: target
    start_seq(et);
  endtask
endclass : i2c_en_test
