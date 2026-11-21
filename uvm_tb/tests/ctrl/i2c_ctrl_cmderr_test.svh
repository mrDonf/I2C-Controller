// -----------------------------------------------------------------------------
// i2c_ctrl_cmderr_test  [VP-CTL-11]
//   command errors E1..E5 in random order (idle / owning the bus), legal READ+NACK then Sr (4.4.5)
// -----------------------------------------------------------------------------
class i2c_ctrl_cmderr_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_cmderr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ctrl_cmderr_vseq      ce = i2c_ctrl_cmderr_vseq::type_id::create("ce");
    i2c_ctrl_nack_rstart_vseq nr = i2c_ctrl_nack_rstart_vseq::type_id::create("nr");
    i2c_ip_ctrl_xfer_vseq     x  = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    i2c_ip_cmderr_e kinds[$];
    i2c_ip_cmderr_e k;
    k = k.first();
    do begin kinds.push_back(k); k = k.next(); end while (k != k.first());
    kinds.shuffle();
    foreach (kinds[i]) begin
      if (!ce.randomize() with { kind == local::kinds[i]; }) `uvm_fatal("TEST", "randomize")
      start_seq(ce);
    end
    start_seq(nr);
    rand_start(x);
  endtask
endclass : i2c_ctrl_cmderr_test
