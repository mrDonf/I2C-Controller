// -----------------------------------------------------------------------------
// i2c_ip_base_test - base of every test
//
//   build_phase      environment configuration (all agents) -> configure_env() hook -> env
//   reset_phase      waits for PRESETn released
//   configure_phase  configure_dut(): default = layer-1 i2c_ip_init_vseq randomized by randomize_init()
//                    (spec 6.2 initialisation with the knobs init_*); do_init = 0 leaves the DUT at reset values
//   main_phase       run_scenario(): the test body - every test composes its own sequences here
//   shutdown_phase   end_of_test(): layer-1 i2c_ip_eot_vseq (bus released, nothing active, FIFOs empty)
//   report_phase     PASS / FAIL banner from the UVM report counts (UVM_ERROR / UVM_FATAL)
//
//   Helpers: start_seq() starts a sequence on the virtual sequencer; rand_start() randomizes then starts it.
// -----------------------------------------------------------------------------
class i2c_ip_base_test extends uvm_test;
  `uvm_component_utils(i2c_ip_base_test)

  i2c_ip_env     env;
  i2c_ip_env_cfg cfg;

  // ---- knobs of the default DUT configuration (configure_phase)
  bit            do_init      = 1;
  i2c_ip_speed_e init_speed   = SPD_FM;
  bit            init_ctrl    = 1;
  bit            init_tgt     = 1;
  bit [3:0]      init_tcfg    = 0;
  bit [21:0]     init_intr_en = 0;
  // ---- end of test
  bit            eot_check    = 1;
  bit            eot_fifos    = 1;
  realtime       test_timeout = 300ms;
  realtime       drain_time   = 20us;

  int unsigned   depth;            // FIFO_DEPTH of the DUT instance

  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  // ================================================================ hooks for the derived tests
  // change the environment configuration before the env is built
  virtual function void configure_env(); endfunction
  // constraints of the initialisation (override to randomize more)
  virtual function bit randomize_init(i2c_ip_init_vseq s);
    return s.randomize() with { speed == local::init_speed; ctrl_en == local::init_ctrl; tgt_en == local::init_tgt;
                                tcfg == local::init_tcfg; intr_en == local::init_intr_en; };
  endfunction
  // DUT configuration
  virtual task configure_dut();
    i2c_ip_init_vseq s;
    if (!do_init) return;
    s = i2c_ip_init_vseq::type_id::create("init");
    if (!randomize_init(s)) `uvm_fatal("TEST", "randomization of the initialisation failed")
    start_seq(s);
  endtask
  // test body
  virtual task run_scenario(); endtask
  // end-of-test checks
  virtual task end_of_test();
    i2c_ip_eot_vseq s = i2c_ip_eot_vseq::type_id::create("eot");
    s.chk_fifos = eot_fifos;
    start_seq(s);
  endtask

  // ================================================================ helpers
  task start_seq(uvm_sequence_base s);
    s.start(env.vsqr);
  endtask

  // integer plusarg with a default (e.g. +N_ITER=50)
  function int unsigned plusarg_int(string name, int unsigned dflt);
    int unsigned v;
    if ($value$plusargs({name, "=%d"}, v)) return v;
    return dflt;
  endfunction

  task rand_start(uvm_sequence_base s);
    if (!s.randomize()) `uvm_fatal("TEST", {"randomization failed: ", s.get_name()})
    s.start(env.vsqr);
  endtask

  // ================================================================ phases
  function void build_phase(uvm_phase phase);
    i2c_vif_t v;
    super.build_phase(phase);
    cfg = i2c_ip_env_cfg::type_id::create("cfg");
    if (!uvm_config_db #(i2c_ip_tb_params)::get(this, "", "tb_params", cfg.p))        `uvm_fatal("TEST", "no tb_params")
    if (!uvm_config_db #(side_vif_t)::get(this, "", "side_vif", cfg.side_vif))         `uvm_fatal("TEST", "no side_vif")
    if (!uvm_config_db #(virtual vip_clk_rst_if)::get(this, "", "crif", cfg.crif))     `uvm_fatal("TEST", "no crif")
    if (!uvm_config_db #(i2c_vif_t)::get(this, "", "i2c_vif", v))                      `uvm_fatal("TEST", "no i2c_vif")
    depth = cfg.p.fifo_depth;

    // APB requester (CPU + DMAC)
    cfg.apb_cfg = apb_agent_cfg::type_id::create("apb_cfg");
    if (!uvm_config_db #(apb_vif_t)::get(this, "", "apb_vif", cfg.apb_cfg.vif))        `uvm_fatal("TEST", "no apb_vif")
    cfg.apb_cfg.role = VIP_MASTER;
    cfg.apb_cfg.timeout_cycles = 16;                                                   // PREADY is always 1 (zero wait)
    cfg.apb_cfg.coverage_en = 0;                                                       // IP coverage: i2c_ip_coverage.cg_apb

    // I2C: VIP master + two VIP slaves (EEPROM models)
    cfg.i2c_mst_cfg = i2c_agent_cfg::type_id::create("i2c_mst_cfg");
    cfg.i2c_mst_cfg.vif = v; cfg.i2c_mst_cfg.role = VIP_MASTER;
    foreach (cfg.slv_addr[i]) begin
      i2c_agent_cfg s = i2c_agent_cfg::type_id::create($sformatf("i2c_slv%0d_cfg", i));
      s.vif = v; s.role = VIP_SLAVE; s.address = cfg.slv_addr[i]; s.coverage_en = 0;
      cfg.i2c_slv_cfg.push_back(s);
    end

    // bit-bang agent, DMAC agent
    cfg.bb_cfg = i2c_bb_cfg::type_id::create("bb_cfg");
    if (!uvm_config_db #(virtual i2c_bb_if)::get(this, "", "bb_vif", cfg.bb_cfg.vif))    `uvm_fatal("TEST", "no bb_vif")
    cfg.bb_cfg.bus = v;
    cfg.dma_cfg = i2c_dma_cfg::type_id::create("dma_cfg");
    if (!uvm_config_db #(virtual i2c_ip_dma_if)::get(this, "", "dma_vif", cfg.dma_cfg.vif)) `uvm_fatal("TEST", "no dma_vif")

    cfg.set_vip_speed(SPD_FM);
    configure_env();
    uvm_config_db #(i2c_ip_env_cfg)::set(this, "env", "cfg", cfg);
    env = i2c_ip_env::type_id::create("env", this);
  endfunction

  function void end_of_elaboration_phase(uvm_phase phase);
    if (uvm_report_enabled(UVM_HIGH)) uvm_top.print_topology();
  endfunction

  // watchdog (no objection: it ends with the run-time phases); +TEST_TIMEOUT_US=<n> overrides test_timeout
  task run_phase(uvm_phase phase);
    realtime t = test_timeout;
    int unsigned us;
    if ($value$plusargs("TEST_TIMEOUT_US=%d", us)) t = us * 1us;
    #(t);
    `uvm_fatal("TEST_TIMEOUT", $sformatf("test not finished within %0t", t))
  endtask

  task reset_phase(uvm_phase phase);
    phase.raise_objection(this);
    wait (cfg.crif.rst_n === 1'b1);
    repeat (5) @(posedge cfg.crif.clk);
    phase.drop_objection(this);
  endtask

  task configure_phase(uvm_phase phase);
    phase.raise_objection(this);
    configure_dut();
    phase.drop_objection(this);
  endtask

  task main_phase(uvm_phase phase);
    phase.raise_objection(this);
    `uvm_info("TEST", $sformatf("%s: scenario start (FIFO_DEPTH %0d)", get_type_name(), depth), UVM_LOW)
    run_scenario();
    phase.drop_objection(this);
  endtask

  task shutdown_phase(uvm_phase phase);
    phase.raise_objection(this);
    if (eot_check) end_of_test();
    #(drain_time);
    phase.drop_objection(this);
  endtask

  function void report_phase(uvm_phase phase);
    uvm_report_server rs = uvm_report_server::get_server();
    int n = rs.get_severity_count(UVM_ERROR) + rs.get_severity_count(UVM_FATAL);
    if (n == 0) `uvm_info("TEST", "\n*** TEST PASSED ***", UVM_NONE)
    else        `uvm_info("TEST", $sformatf("\n*** TEST FAILED (%0d error(s)) ***", n), UVM_NONE)
  endfunction
endclass : i2c_ip_base_test

// ---------------------------------------------------------------- feature-group base tests
// registers / APB: the DUT stays at its reset values (no initialisation)
class i2c_reg_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_reg_base_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    do_init = 0;
  endfunction
endclass : i2c_reg_base_test

// controller: controller enabled, target disabled
class i2c_ctrl_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_ctrl_base_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 1; init_tgt = 0;
  endfunction
endclass : i2c_ctrl_base_test

// target: target enabled, controller disabled
class i2c_tgt_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_tgt_base_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 0; init_tgt = 1;
  endfunction
endclass : i2c_tgt_base_test

// bus level: both roles enabled
class i2c_bus_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_bus_base_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 1; init_tgt = 1;
  endfunction
endclass : i2c_bus_base_test

// DMA: controller enabled, DMA_DONE routed to irq_o
class i2c_dma_base_test extends i2c_ip_base_test;
  `uvm_component_utils(i2c_dma_base_test)
  function new(string name, uvm_component parent);
    super.new(name, parent);
    init_ctrl = 1; init_tgt = 0; init_intr_en = 22'h20_0000;
  endfunction
endclass : i2c_dma_base_test
