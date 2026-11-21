// -----------------------------------------------------------------------------
// i2c_ctrl_irq_test  [VP-CTL-04]
//   interrupt-driven completion: irq_ctrl_o raised by CMPL, released by the W1C (6.12)
// -----------------------------------------------------------------------------
class i2c_ctrl_irq_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_irq_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_intr_en = 22'h0C_003C;                   // interrupts of 6.12
  endfunction

  virtual task run_scenario();
    i2c_ctrl_cmpl_irq_vseq irq = i2c_ctrl_cmpl_irq_vseq::type_id::create("irq");
    i2c_ip_ctrl_xfer_vseq  x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    repeat (3) begin
      start_seq(irq);
      rand_start(x);
    end
  endtask
endclass : i2c_ctrl_irq_test
