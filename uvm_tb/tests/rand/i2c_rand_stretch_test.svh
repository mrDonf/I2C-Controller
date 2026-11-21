// -----------------------------------------------------------------------------
// i2c_rand_stretch_test  [VP-RND-04]
//   random traffic with random target clock stretching: factory type override of the controller transfer
// -----------------------------------------------------------------------------
class i2c_rand_stretch_test extends i2c_rand_test;
  `uvm_component_utils(i2c_rand_stretch_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    // every i2c_ip_ctrl_xfer_vseq created by the random sequence becomes a stretching one
    set_type_override_by_type(i2c_ip_ctrl_xfer_vseq::get_type(), i2c_ip_ctrl_xfer_stretch_vseq::get_type());
  endfunction

  virtual task run_scenario();
    super.run_scenario();
  endtask
endclass : i2c_rand_stretch_test
