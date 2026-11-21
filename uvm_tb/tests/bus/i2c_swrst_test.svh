// -----------------------------------------------------------------------------
// i2c_swrst_test  [VP-BUS-11]
//   SWRST: cleared / kept state of 5.2.1, SWRST in the middle of a transfer, recovery (5.2.1, 6.1.2)
// -----------------------------------------------------------------------------
class i2c_swrst_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_swrst_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_swrst_regs_vseq   regs = i2c_swrst_regs_vseq::type_id::create("regs");
    i2c_swrst_active_vseq act  = i2c_swrst_active_vseq::type_id::create("act");
    i2c_ip_ctrl_xfer_vseq x    = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    start_seq(regs);
    init_ctrl = 1; init_tgt = 0;
    configure_dut();                       // re-initialise: controller
    rand_start(act);
    rand_start(x);
  endtask
endclass : i2c_swrst_test
