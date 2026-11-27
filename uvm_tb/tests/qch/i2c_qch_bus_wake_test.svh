// -----------------------------------------------------------------------------
// i2c_qch_bus_wake_test  [VP-QCH-04]
//   external messages arriving while the IP is stopped, at every F/S speed
// -----------------------------------------------------------------------------
class i2c_qch_bus_wake_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_bus_wake_test)
  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  virtual task run_scenario();
    i2c_ip_speed_e sp[3] = '{SPD_SM, SPD_FMP, SPD_FM};
    foreach (sp[i]) begin
      i2c_ip_init_vseq      init = i2c_ip_init_vseq::type_id::create($sformatf("init_%s", sp[i].name()));
      i2c_qch_bus_wake_vseq bw   = i2c_qch_bus_wake_vseq::type_id::create($sformatf("bw_%s", sp[i].name()));
      if (!init.randomize() with { speed == local::sp[i]; ctrl_en == 1; tgt_en == 1; tcfg == 0; intr_en == 0; })
        `uvm_fatal("TEST", "init")
      start_seq(init);
      if (!bw.randomize() with { n_msg == ((sp[i] == SPD_SM) ? 3 : 6); }) `uvm_fatal("TEST", "bw")
      start_seq(bw);
    end
  endtask
endclass : i2c_qch_bus_wake_test
