// -----------------------------------------------------------------------------
// i2c_ctrl_hs_test  [VP-CTL-16]
//   Hs-mode controller: controller code, Hs timing, hs_mode_o / scl_cs_en_o, HS_EN priority, back to F/S (4.4.7,
//   6.6)
// -----------------------------------------------------------------------------
class i2c_ctrl_hs_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_hs_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_hs_cfg_vseq    hs   = i2c_ip_hs_cfg_vseq::type_id::create("hs");
    i2c_ctrl_hs_vseq      xfer = i2c_ctrl_hs_vseq::type_id::create("xfer");
    i2c_ctrl_hs_prio_vseq prio = i2c_ctrl_hs_prio_vseq::type_id::create("prio");
    rand_start(hs);                        // random controller code 0000_1xxx
    rand_start(xfer);
    start_seq(prio);
  endtask
endclass : i2c_ctrl_hs_test
