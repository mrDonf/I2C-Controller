// -----------------------------------------------------------------------------
// apb_test_pkg - environment-level tests of the APB VIP
//   Back-to-back (master agent <-> slave agent with memory):  smoke, rand, waits, b2b, strobe, error, reset, checker
//   Against a real DUT (GPIO of this repository, apb_dut_tb_top):  dut
// -----------------------------------------------------------------------------
`ifndef APB_TEST_PKG_SV
`define APB_TEST_PKG_SV

package apb_test_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
`include "vip_macros.svh"
  import vip_common_pkg::*;
  import apb_pkg::*;

  typedef virtual vip_clk_rst_if crif_t;

  class apb_base_test extends uvm_test;
    `uvm_component_utils(apb_base_test)

    apb_env       env;
    apb_env_cfg   ecfg;
    apb_agent_cfg mcfg, scfg;
    crif_t        crif;
    vip_expect_catcher catcher;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    // tests override these two hooks
    virtual function void configure();            endfunction
    virtual function bit  with_slave();  return 1; endfunction

    function void build_phase(uvm_phase phase);
      apb_vif_t vif;
      super.build_phase(phase);
      if (!uvm_config_db #(apb_vif_t)::get(this, "", "vif", vif)) `vip_fatal("TEST", "no vif")
      if (!uvm_config_db #(crif_t)::get(this, "", "crif", crif)) `vip_fatal("TEST", "no crif")
      catcher = new("catcher");
      ecfg = apb_env_cfg::type_id::create("ecfg");
      mcfg = apb_agent_cfg::type_id::create("mcfg");
      mcfg.vif = vif; mcfg.role = VIP_MASTER;
      ecfg.mst_cfg = mcfg;
      if (with_slave()) begin
        scfg = apb_agent_cfg::type_id::create("scfg");
        scfg.vif = vif; scfg.role = VIP_SLAVE;
        ecfg.slv_cfg = scfg;
      end else ecfg.has_scoreboard = 0;
      configure();
      uvm_config_db #(apb_env_cfg)::set(this, "env", "cfg", ecfg);
      env = apb_env::type_id::create("env", this);
    endfunction

    virtual task run_seqs(uvm_phase phase);       endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      wait (crif.rst_n === 1'b1);
      repeat (3) @(posedge crif.clk);
      run_seqs(phase);
      repeat (10) @(posedge crif.clk);
      phase.drop_objection(this);
    endtask

    function void check_phase(uvm_phase phase);
      apb_vif_t v = mcfg.vif;
      if (v.viol_cnt != 0 && !v.expect_violations)
        `vip_error("APB_TEST", $sformatf("%0d protocol violations (last %s)", v.viol_cnt, v.viol_last))
      if (env.sb != null && env.sb.n_mismatch != 0) `vip_error("APB_TEST", "scoreboard mismatches")
    endfunction
  endclass : apb_base_test

  // ---------------------------------------------------------------- smoke
  class apb_smoke_test extends apb_base_test;
    `uvm_component_utils(apb_smoke_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      apb_wr_rd_seq s = apb_wr_rd_seq::type_id::create("s");
      if (!s.randomize() with { n == 30; lo == 0; hi == 'h3FF; }) `vip_fatal("TEST", "randomize")
      s.start(env.mst.sqr);
      `vip_info("TEST", $sformatf("monitor: %0d transfers", env.mst.mon.n_xfer), UVM_LOW)
      if (env.mst.mon.n_xfer != 60) `vip_error("APB_TEST", $sformatf("expected 60 transfers, monitor saw %0d", env.mst.mon.n_xfer))
    endtask
  endclass : apb_smoke_test

  // ---------------------------------------------------------------- random traffic, random waits and errors
  class apb_rand_test extends apb_base_test;
    `uvm_component_utils(apb_rand_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure();
      scfg.wait_min = 0; scfg.wait_max = 4;
      scfg.default_pready = $urandom_range(1);
      scfg.err_pct = 5;                               // scoreboard accepts errors because the config allows them
    endfunction
    task run_seqs(uvm_phase phase);
      apb_rand_seq r = apb_rand_seq::type_id::create("r");
      apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
      if (!r.randomize() with { n == 300; lo == 0; hi == 'hFFF; }) `vip_fatal("TEST", "randomize")
      r.start(env.mst.sqr);
      if (!w.randomize() with { n == 100; lo == 'h1000; hi == 'h1FFF; }) `vip_fatal("TEST", "randomize")
      scfg.err_pct = 0;
      w.start(env.mst.sqr);
    endtask
  endclass : apb_rand_test

  // ---------------------------------------------------------------- wait states
  class apb_wait_test extends apb_base_test;
    `uvm_component_utils(apb_wait_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      int unsigned lim[4] = '{0, 1, 5, 16};
      foreach (lim[i]) begin
        apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
        scfg.wait_min = lim[i] / 2; scfg.wait_max = lim[i];
        if (!w.randomize() with { n == 40; lo == 0; hi == 'hFFF; }) `vip_fatal("TEST", "randomize")
        w.start(env.mst.sqr);
      end
      if (env.mst.mon.n_wait_cycles == 0) `vip_error("APB_TEST", "no wait states observed")
    endtask
  endclass : apb_wait_test

  // ---------------------------------------------------------------- back-to-back transfers
  class apb_b2b_test extends apb_base_test;
    `uvm_component_utils(apb_b2b_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      for (int k = 0; k < 4; k++) begin
        apb_burst_seq b = apb_burst_seq::type_id::create("b");
        scfg.wait_max = (k < 2) ? 0 : 3;
        if (!b.randomize() with { n == 32; base == 'h2000 + k * 'h100; write_burst == 1; }) `vip_fatal("TEST", "randomize")
        b.start(env.mst.sqr);
        if (!b.randomize() with { n == 32; base == 'h2000 + k * 'h100; write_burst == 0; }) `vip_fatal("TEST", "randomize")
        b.start(env.mst.sqr);
      end
      `vip_info("TEST", $sformatf("%0d transfers", env.mst.mon.n_xfer), UVM_LOW)
    endtask
    function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (env.mst.mon.n_b2b < 100)
        `vip_error("APB_TEST", $sformatf("only %0d back-to-back transfers (idle = 0) observed by the monitor", env.mst.mon.n_b2b))
    endfunction
  endclass : apb_b2b_test

  // ---------------------------------------------------------------- byte strobes
  class apb_strobe_test extends apb_base_test;
    `uvm_component_utils(apb_strobe_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      repeat (20) begin
        apb_strobe_seq s = apb_strobe_seq::type_id::create("s");
        if (!s.randomize() with { addr[1:0] == 0; addr < 'h400; }) `vip_fatal("TEST", "randomize")
        s.start(env.mst.sqr);
      end
    endtask
  endclass : apb_strobe_test

  // ---------------------------------------------------------------- PSLVERR handling
  class apb_error_test extends apb_base_test;
    `uvm_component_utils(apb_error_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure();
      scfg.err_lo = 'h100; scfg.err_hi = 'h1FF;
      scfg.base_addr = 0; scfg.size = 'h1000; scfg.unmapped_error = 1;
    endfunction
    task run_seqs(uvm_phase phase);
      apb_error_seq e = apb_error_seq::type_id::create("e");
      apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
      e.err_addr = 'h140; e.ok_addr = 'h200;
      e.start(env.mst.sqr);
      e.err_addr = 'h2000; e.ok_addr = 'h204;               // unmapped address
      e.start(env.mst.sqr);
      if (!w.randomize() with { n == 50; lo == 'h200; hi == 'hFFF; }) `vip_fatal("TEST", "randomize")
      w.start(env.mst.sqr);
      if (env.mst.mon.n_err < 4) `vip_error("APB_TEST", "PSLVERR was not observed")
    endtask
  endclass : apb_error_test

  // ---------------------------------------------------------------- reset in the middle of traffic
  class apb_reset_test extends apb_base_test;
    `uvm_component_utils(apb_reset_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure();
      scfg.wait_min = 1; scfg.wait_max = 4;
    endfunction
    task run_seqs(uvm_phase phase);
      for (int k = 0; k < 3; k++) begin
        apb_rand_seq r = apb_rand_seq::type_id::create("r");
        apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
        if (!r.randomize() with { n == 200; lo == 0; hi == 'hFF; }) `vip_fatal("TEST", "randomize")
        fork
          r.start(env.mst.sqr);
          begin
            crif.wait_cycles($urandom_range(30, 120));
            crif.pulse($urandom_range(1, 4));
          end
        join
        // the memory model keeps its content but the interrupted transfers are undefined: restart from scratch
        scfg.mem.clear(); env.sb.ref_mem.clear();
        if (!w.randomize() with { n == 30; lo == 0; hi == 'hFF; }) `vip_fatal("TEST", "randomize")
        w.start(env.mst.sqr);
      end
    endtask
  endclass : apb_reset_test

  // ---------------------------------------------------------------- the interface assertions fire when the protocol is broken
  class apb_checker_test extends apb_base_test;
    `uvm_component_utils(apb_checker_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure();
      scfg.wait_min = 2; scfg.wait_max = 4;
      ecfg.has_scoreboard = 0;
    endfunction
    task run_seqs(uvm_phase phase);
      apb_inject_e kinds[4] = '{APB_INJ_NO_ACCESS, APB_INJ_EARLY_PENABLE, APB_INJ_ADDR_CHANGE, APB_INJ_STRB_ON_READ};
      string       ids[4]   = '{"APB_A_ACCESS", "APB_A_SETUP", "APB_A_STABLE", "APB_A_RD_STRB"};
      apb_vif_t v = mcfg.vif;
      v.expect_violations = 1;
      mcfg.timeout_cycles = 30;                          // a requester that breaks the protocol is not answered
      catcher.add_expect("APB_TIMEOUT", UVM_ERROR, 0);
      catcher.add_expect("APB_SEQ_MISMATCH", UVM_ERROR, 0);
      catcher.install();
      foreach (kinds[i]) begin
        int unsigned cnt_pre = v.viol_by_id.exists(ids[i]) ? v.viol_by_id[ids[i]] : 0;
        apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
        mcfg.inject = kinds[i]; mcfg.inject_pct = 100;
        if (!w.randomize() with { n == 6; lo == 0; hi == 'hFF; }) `vip_fatal("TEST", "randomize")
        w.start(env.mst.sqr);
        mcfg.inject = APB_INJ_NONE;
        crif.wait_cycles(20);
        if ((v.viol_by_id.exists(ids[i]) ? v.viol_by_id[ids[i]] : 0) <= cnt_pre)
          `vip_error("APB_TEST", $sformatf("injection %s did not trigger assertion %s", kinds[i].name(), ids[i]))
        else `vip_info("TEST", $sformatf("%s detected by %s (%0d times)", kinds[i].name(), ids[i], v.viol_by_id[ids[i]] - cnt_pre), UVM_LOW)
        crif.pulse(2);                                   // recover from a half-finished transfer
        crif.wait_cycles(5);
      end
      // clean traffic must not add violations
      begin
        int unsigned c0 = v.viol_cnt;
        apb_wr_rd_seq w = apb_wr_rd_seq::type_id::create("w");
        if (!w.randomize() with { n == 40; lo == 0; hi == 'hFF; }) `vip_fatal("TEST", "randomize")
        w.start(env.mst.sqr);
        if (v.viol_cnt != c0) `vip_error("APB_TEST", "violations reported for legal traffic")
      end
    endtask
    function void check_phase(uvm_phase phase);
      // violations are expected here, the base check must not complain
    endfunction
  endclass : apb_checker_test

  // ---------------------------------------------------------------- real DUT: GPIO register file (apb_dut_tb_top)
  class apb_dut_seq extends apb_base_seq;
    `uvm_object_utils(apb_dut_seq)
    function new(string name = "apb_dut_seq"); super.new(name); endfunction
    task body();
      bit [31:0] d;
      bit e;
      // ID register: read-only, fixed value, a write is rejected
      read('h000, d, e);
      if (e || d[31:16] != 16'h4750) `vip_error("APB_TEST", $sformatf("GPIO ID wrong: 0x%0h err=%0b", d, e))
      write('h000, 32'h1234, e);
      if (!e) `vip_error("APB_TEST", "write to the read-only ID register did not report PSLVERR")
      read('h400, d, e);
      if (!e) `vip_error("APB_TEST", "read of an unmapped address did not report PSLVERR")
    endtask
  endclass : apb_dut_seq

  class apb_dut_test extends apb_base_test;
    `uvm_component_utils(apb_dut_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function bit with_slave(); return 0; endfunction
    task run_seqs(uvm_phase phase);
      apb_dut_seq      ds = apb_dut_seq::type_id::create("ds");
      apb_reg_walk_seq w  = apb_reg_walk_seq::type_id::create("w");
      apb_strobe_seq   s  = apb_strobe_seq::type_id::create("s");
      apb_burst_seq    b  = apb_burst_seq::type_id::create("b");
      ds.start(env.mst.sqr);
      // plain read/write registers of the GPIO map: DATA_OUT, DIR, INT_TYPE, INT_POL, INT_ENABLE
      w.add('h008); w.add('h00C); w.add('h01C); w.add('h020); w.add('h02C);
      w.start(env.mst.sqr);
      // byte lanes of DATA_OUT (0x008)
      if (!s.randomize() with { addr == 'h008; }) `vip_fatal("TEST", "randomize")
      s.start(env.mst.sqr);
      // back-to-back reads of the whole map (PSEL stays high between the transfers)
      if (!b.randomize() with { n == 16; base == 0; write_burst == 0; }) `vip_fatal("TEST", "randomize")
      b.start(env.mst.sqr);
      if (env.mst.mon.n_b2b < 8) `vip_error("APB_TEST", "the burst did not produce back-to-back transfers")
    endtask
  endclass : apb_dut_test

  // ---------------------------------------------------------------------------------------------------------------
  // apb_assert_test - checker verification without drivers: the test drives the pins cycle by cycle, one scenario per
  // checker ID (and legal baselines); every scenario must fire exactly its target ID (vip_viol_check)
  // ---------------------------------------------------------------------------------------------------------------
  class apb_assert_test extends apb_base_test;
    `uvm_component_utils(apb_assert_test)
    apb_vif_t v;
    int unsigned n_scen;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function bit  with_slave(); return 0; endfunction
    function void configure(); mcfg.role = VIP_MONITOR; mcfg.coverage_en = 0; endfunction

    function void start_of_simulation_phase(uvm_phase phase);
      v = mcfg.vif;
      v.expect_violations = 1;
      idle_pins();
    endfunction

    function void idle_pins();
      v.psel = 0; v.penable = 0; v.pwrite = 0; v.paddr = '0; v.pwdata = '0; v.pstrb = '0; v.pprot = '0;
      v.pready = 0; v.prdata = '0; v.pslverr = 0;
    endfunction

    // one clock cycle with these pin values (applied 1 ns after the rising edge)
    task cyc(logic sel, logic en, logic wr, logic [APB_ADDR_W-1:0] a, logic rdy, logic [APB_DATA_W-1:0] wd = 'h1234,
             logic [APB_STRB_W-1:0] st = '1, logic [APB_DATA_W-1:0] rd = 'h5678);
      v.psel = sel; v.penable = en; v.pwrite = wr; v.paddr = a; v.pready = rdy; v.pwdata = wd;
      v.pstrb = wr ? st : '0; v.prdata = rd; v.pslverr = 0;
      @(posedge v.pclk); #1ns;
    endtask

    task idle(int unsigned n = 3);
      idle_pins();
      repeat (n) begin @(posedge v.pclk); #1ns; end
    endtask

    task expect_only(string target, vip_viol_map_t snap, string allowed[$] = {});
      string m;
      idle();
      m = vip_viol_check(target, snap, v.viol_by_id, allowed);
      n_scen++;
      if (m != "") `vip_error("APB_ASSERT", $sformatf("scenario %0d (%s):%s", n_scen, (target == "") ? "legal" : target, m))
      else `vip_info("APB_ASSERT", $sformatf("scenario %0d: %s", n_scen, (target == "") ? "legal traffic, no violation" : {target, " fired alone"}), UVM_LOW)
    endtask

    task run_seqs(uvm_phase phase);
      vip_viol_map_t b;
      @(posedge v.pclk); #1ns;
      // legal: write with a wait state, read, back-to-back transfers
      b = v.viol_by_id;
      cyc(1, 0, 1, 'h10, 0); cyc(1, 1, 1, 'h10, 0); cyc(1, 1, 1, 'h10, 1);
      cyc(1, 0, 0, 'h14, 0); cyc(1, 1, 0, 'h14, 1);
      cyc(1, 0, 1, 'h18, 0); cyc(1, 1, 1, 'h18, 1);
      expect_only("", b);
      // APB_A_SETUP: PENABLE in the first cycle of a transfer
      b = v.viol_by_id; cyc(1, 1, 1, 'h20, 1); expect_only("APB_A_SETUP", b);
      // APB_A_ACCESS: SETUP followed by another SETUP cycle
      b = v.viol_by_id; cyc(1, 0, 1, 'h24, 0); cyc(1, 0, 1, 'h24, 0); cyc(1, 1, 1, 'h24, 1); expect_only("APB_A_ACCESS", b);
      // APB_A_PEN_PSEL: PENABLE without PSEL
      b = v.viol_by_id; cyc(0, 1, 0, 'h0, 0); expect_only("APB_A_PEN_PSEL", b);
      // APB_A_STABLE: address changes in a wait state / between SETUP and ACCESS
      b = v.viol_by_id; cyc(1, 0, 1, 'h28, 0); cyc(1, 1, 1, 'h28, 0); cyc(1, 1, 1, 'h2C, 1); expect_only("APB_A_STABLE", b);
      b = v.viol_by_id; cyc(1, 0, 1, 'h30, 0); cyc(1, 1, 1, 'h30, 0, 'h9999); cyc(1, 1, 1, 'h30, 1, 'h9999); expect_only("APB_A_STABLE", b);
      b = v.viol_by_id; cyc(1, 0, 0, 'h34, 0); cyc(1, 1, 0, 'h38, 1); expect_only("APB_A_STABLE", b);
      // APB_A_PEN_END: PENABLE stays high after the completing cycle
      b = v.viol_by_id; cyc(1, 0, 1, 'h3C, 0); cyc(1, 1, 1, 'h3C, 1); cyc(1, 1, 1, 'h3C, 1); expect_only("APB_A_PEN_END", b);
      // APB_A_RD_STRB: PSTRB in a read
      b = v.viol_by_id;
      v.psel = 1; v.penable = 0; v.pwrite = 0; v.paddr = 'h40; v.pstrb = 'h1; v.pready = 0; @(posedge v.pclk); #1ns;
      v.penable = 1; v.pready = 1; @(posedge v.pclk); #1ns;
      expect_only("APB_A_RD_STRB", b);
      // APB_A_NO_X: unknown PSEL, PADDR in a transfer, PREADY in ACCESS, PRDATA at the end of a read, PWDATA in a write
      b = v.viol_by_id; v.psel = 'x; @(posedge v.pclk); #1ns; expect_only("APB_A_NO_X", b);
      b = v.viol_by_id; cyc(1, 0, 1, 'x, 0); cyc(1, 1, 1, 'x, 1); expect_only("APB_A_NO_X", b);
      b = v.viol_by_id; cyc(1, 0, 1, 'h44, 0); cyc(1, 1, 1, 'h44, 'x); cyc(1, 1, 1, 'h44, 1); expect_only("APB_A_NO_X", b);
      b = v.viol_by_id; cyc(1, 0, 0, 'h48, 0); cyc(1, 1, 0, 'h48, 1, 'h1234, '0, 'x); expect_only("APB_A_NO_X", b);
      // legal: PWDATA is a don't-care in a read and may change
      b = v.viol_by_id; cyc(1, 0, 0, 'h48, 0, 'h1); cyc(1, 1, 0, 'h48, 0, 'h2); cyc(1, 1, 0, 'h48, 1, 'h3); expect_only("", b);
      b = v.viol_by_id; cyc(1, 0, 1, 'h4C, 0, 'x); cyc(1, 1, 1, 'h4C, 1, 'x); expect_only("APB_A_NO_X", b);
      // a violation during reset is not reported (checks are disabled while PRESETn is low)
      b = v.viol_by_id;
      fork crif.pulse(4); join_none
      @(negedge v.presetn); #1ns;
      v.penable = 1; v.psel = 0;
      @(posedge v.presetn); #1ns; idle_pins();
      expect_only("", b);
      // legal traffic again
      b = v.viol_by_id; cyc(1, 0, 1, 'h50, 0); cyc(1, 1, 1, 'h50, 1); cyc(1, 0, 0, 'h50, 0); cyc(1, 1, 0, 'h50, 0); cyc(1, 1, 0, 'h50, 1); expect_only("", b);
      if (n_scen != 17) `vip_error("APB_ASSERT", $sformatf("%0d scenarios run", n_scen))
    endtask

    function void check_phase(uvm_phase phase); endfunction
  endclass : apb_assert_test

  // ---------------------------------------------------------------- attributes and the whole address space
  // every PPROT value in both directions, addresses in all four quarters of the address space, wait states
  class apb_attr_test extends apb_base_test;
    `uvm_component_utils(apb_attr_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure(); scfg.wait_max = 3; endfunction
    task run_seqs(uvm_phase phase);
      apb_api_seq b = apb_api_seq::type_id::create("b");
      fork b.start(env.mst.sqr); join_none
      #0;
      for (int q = 0; q < 4; q++) for (int pr = 0; pr < 8; pr++) begin
        apb_item w = apb_item::type_id::create("w"), r = apb_item::type_id::create("r");
        if (!w.randomize() with { write == 1; addr[APB_ADDR_W-1 -: 2] == q; }) `vip_fatal("TEST", "randomize")
        w.prot = pr;
        b.xfer(w);
        if (!r.randomize() with { write == 0; addr == w.addr; }) `vip_fatal("TEST", "randomize")
        r.prot = 7 - pr;
        b.xfer(r);
      end
      if (env.sb.n_mismatch != 0) `vip_error("APB_TEST", "scoreboard mismatches")
    endtask
  endclass : apb_attr_test

endpackage : apb_test_pkg

`endif
