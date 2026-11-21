// -----------------------------------------------------------------------------
// i2c_tgt_rand_addr_test  [VP-TGT-15]
//   corner: random own addresses (whole legal range), writes / serviced reads to both
// -----------------------------------------------------------------------------
class i2c_tgt_rand_addr_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_tgt_rand_addr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual function bit randomize_init(i2c_ip_init_vseq s);
    return s.randomize() with { ctrl_en == 0; tgt_en == 1; tcfg == 0; intr_en == 0; };
  endfunction

  virtual task run_scenario();
    i2c_tgt_write_vseq         w = i2c_tgt_write_vseq::type_id::create("w");
    i2c_tgt_serviced_read_vseq r = i2c_tgt_serviced_read_vseq::type_id::create("r");
    for (int k = 0; k < 3; k++) begin
      if (k > 0) configure_dut();          // new random addresses
      repeat (2) rand_start(w);
      rand_start(r);
    end
  endtask
endclass : i2c_tgt_rand_addr_test
