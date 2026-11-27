// -----------------------------------------------------------------------------
// i2c_qch_rand_test  [VP-QCH-08]
//   random controller / target traffic (bursts separated by idle periods) with a background clock controller
//   issuing random quiescence requests; data checked by the scoreboard
// -----------------------------------------------------------------------------
class i2c_qch_rand_test extends i2c_qch_base_test;
  `uvm_component_utils(i2c_qch_rand_test)
  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  virtual task run_scenario();
    i2c_ip_qch_bg_vseq bg = i2c_ip_qch_bg_vseq::type_id::create("bg");
    int unsigned n = plusarg_int("N_ITER", 25);
    if (!bg.randomize() with { gap_min_ns inside {[200:2000]}; gap_max_ns inside {[5000:40000]}; }) `uvm_fatal("TEST", "bg")
    fork
      start_seq(bg);
      begin
        // traffic in bursts of random transfers separated by idle periods (the requests are denied during a burst)
        for (int k = 0; k < n; k += 5) begin
          i2c_rand_traffic_vseq tr = i2c_rand_traffic_vseq::type_id::create($sformatf("traffic%0d", k));
          int unsigned m = (n - k < 5) ? n - k : 5;
          if (!tr.randomize() with { n_iter == m; max_len inside {[4:24]}; }) `uvm_fatal("TEST", "traffic")
          start_seq(tr);
          #($urandom_range(60, 10) * 1us);
        end
        bg.stop_req = 1;
      end
    join
    chk_counts(bg);
  endtask

  function void chk_counts(i2c_ip_qch_bg_vseq bg);
    if (bg.n_acc == 0) `uvm_error("TEST", "no Q-Channel request accepted during the random run")
    `uvm_info("TEST", $sformatf("random run: %0d accepted / %0d denied quiescence requests", bg.n_acc, bg.n_deny), UVM_LOW)
  endfunction
endclass : i2c_qch_rand_test
