// -----------------------------------------------------------------------------
// i2c_rand_test  [VP-RND-01]
//   constrained-random mix of controller and target traffic (+N_ITER, +SPEED)
// -----------------------------------------------------------------------------
class i2c_rand_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_rand_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  int unsigned n_iter;
  int          spd_sel = -1;       // -1: random Fm / Fm+, else i2c_ip_speed_e from +SPEED=SM|FM|FMP
  function void start_of_simulation_phase(uvm_phase phase);
    string s;
    n_iter = plusarg_int("N_ITER", 30);
    if ($value$plusargs("SPEED=%s", s)) spd_sel = (s == "SM") ? int'(SPD_SM) : (s == "FMP") ? int'(SPD_FMP) : int'(SPD_FM);
  endfunction
  // random speed (or +SPEED), random own addresses, general call enabled
  virtual function bit randomize_init(i2c_ip_init_vseq s);
    return s.randomize() with {
      ctrl_en == 1; tgt_en == 1; tcfg == 1; intr_en == 0;
      (local::spd_sel >= 0) -> speed == i2c_ip_speed_e'(local::spd_sel);
      (local::spd_sel <  0) -> speed inside {SPD_FM, SPD_FMP};
    };
  endfunction

  virtual task run_scenario();
    i2c_rand_traffic_vseq s = i2c_rand_traffic_vseq::type_id::create("rnd");
    if (!s.randomize() with { n_iter == local::n_iter; max_len == 2 * local::depth; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
  endtask
endclass : i2c_rand_test
