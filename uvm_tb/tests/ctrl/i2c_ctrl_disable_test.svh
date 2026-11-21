// -----------------------------------------------------------------------------
// i2c_ctrl_disable_test  [VP-CTL-13]
//   CTRL_EN -> 0 while owning the bus = user abort with STOP (4.4.9)
// -----------------------------------------------------------------------------
class i2c_ctrl_disable_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_disable_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_disable_vseq dis = i2c_ctrl_disable_vseq::type_id::create("dis");
    i2c_ip_ctrl_xfer_vseq x   = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    repeat (2) begin
      start_seq(dis);
      rand_start(x);
    end
  endtask
endclass : i2c_ctrl_disable_test
