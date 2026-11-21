// -----------------------------------------------------------------------------
// i2c_reg_presetn_test  [VP-REG-02]
//   PRESETn in the middle of activity: registers, FIFOs and outputs return to reset (6.1.1)
// -----------------------------------------------------------------------------
class i2c_reg_presetn_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_reg_presetn_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ctrl_xfer_vseq  xfer  = i2c_ip_ctrl_xfer_vseq::type_id::create("xfer");
    i2c_reg_dirty_vseq     dirty = i2c_reg_dirty_vseq::type_id::create("dirty");
    i2c_ip_presetn_vseq    rst_p = i2c_ip_presetn_vseq::type_id::create("rst_p");
    uvm_reg_hw_reset_seq   rst   = uvm_reg_hw_reset_seq::type_id::create("rst");
    i2c_reg_reset_val_vseq vals  = i2c_reg_reset_val_vseq::type_id::create("vals");
    i2c_reset_outputs_vseq outs  = i2c_reset_outputs_vseq::type_id::create("outs");
    rand_start(xfer);                      // some traffic, then a non-reset state
    start_seq(dirty);
    rst_p.cycles = $urandom_range(1, 20);
    start_seq(rst_p);                      // PRESETn pulse
    start_seq(outs);
    rst.model = env.ral;
    rst.start(null);
    start_seq(vals);
  endtask
endclass : i2c_reg_presetn_test
