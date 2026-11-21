// -----------------------------------------------------------------------------
// i2c_rand_reconfig_test  [VP-RND-05]
//   re-initialisation between bursts: random speed (Sm / Fm / Fm+), own addresses, IDLE_VAL, FIFO thresholds
// -----------------------------------------------------------------------------
class i2c_rand_reconfig_test extends i2c_rand_test;
  `uvm_component_utils(i2c_rand_reconfig_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_rand_traffic_vseq s   = i2c_rand_traffic_vseq::type_id::create("rnd");
    i2c_ip_init_vseq      ini = i2c_ip_init_vseq::type_id::create("ini");
    i2c_ip_fifo_thr_vseq  thr = i2c_ip_fifo_thr_vseq::type_id::create("thr");
    for (int k = 0; k < 4; k++) begin
      if (k > 0) begin
        if (!ini.randomize() with { ctrl_en == 1; tgt_en == 1; tcfg == 1; intr_en == 0; }) `uvm_fatal("TEST", "randomize")
        start_seq(ini);
      end
      if (!thr.randomize() with { cmd_thr < local::depth; crx_thr < local::depth; ttx_thr < local::depth; trx_thr < local::depth; })
        `uvm_fatal("TEST", "randomize")
      start_seq(thr);
      if (!s.randomize() with { n_iter == local::n_iter / 3 + 1; max_len == 2 * local::depth; }) `uvm_fatal("TEST", "randomize")
      start_seq(s);
    end
  endtask
endclass : i2c_rand_reconfig_test
