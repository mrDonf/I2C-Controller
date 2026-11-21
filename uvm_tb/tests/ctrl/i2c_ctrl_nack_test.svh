// -----------------------------------------------------------------------------
// i2c_ctrl_nack_test  [VP-CTL-08]
//   address NACK (write / read, queued transfers flushed), own target never answers, irq for NACK (4.4.4, R4,
//   6.12)
// -----------------------------------------------------------------------------
class i2c_ctrl_nack_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_nack_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_tgt = 1;                                  // the own target is enabled (R4)
  endfunction

  virtual task run_scenario();
    i2c_ctrl_addr_nack_vseq an  = i2c_ctrl_addr_nack_vseq::type_id::create("an");
    i2c_ctrl_own_tgt_vseq   own = i2c_ctrl_own_tgt_vseq::type_id::create("own");
    i2c_ctrl_nack_irq_vseq  irq = i2c_ctrl_nack_irq_vseq::type_id::create("irq");
    i2c_ip_ctrl_xfer_vseq   x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    repeat (4) rand_start(an);
    start_seq(own);
    start_seq(irq);
    rand_start(x);                         // a normal transfer after the errors
  endtask
endclass : i2c_ctrl_nack_test
