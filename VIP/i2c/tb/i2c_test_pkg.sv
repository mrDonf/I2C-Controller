// -----------------------------------------------------------------------------
// i2c_test_pkg - tests of the I2C VIP
//   back-to-back (i2c_tb_top: VIP master, VIP slave 0x50 (7 bit), VIP slave 0x2A5 (10 bit), monitor + scoreboard):
//     smoke, speeds (Sm / Fm / Fm+), addr10, nack, stretch, multi (two slaves, repeated STARTs), timing (checker)
//   against the I2C IP of this repository (i2c_dut_tb_top: APB VIP programs the IP master, VIP slave EEPROM 0x50):
//     dut_wr_rd, dut_nack, dut_stretch, dut_timing (Sm and Fm with the UM10204 timing checks of the monitor)
// -----------------------------------------------------------------------------
`ifndef I2C_TEST_PKG_SV
`define I2C_TEST_PKG_SV

package i2c_test_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  `include "vip_macros.svh"
  import vip_common_pkg::*;
  import i2c_pkg::*;
  import apb_pkg::*;

  typedef virtual vip_clk_rst_if crif_t;

  // ================================================================ back-to-back
  class i2c_b2b_base_test extends uvm_test;
    `uvm_component_utils(i2c_b2b_base_test)
    i2c_env       env;
    i2c_env_cfg   ecfg;
    i2c_agent_cfg mcfg, s0cfg, s1cfg;
    crif_t        crif;
    i2c_api_seq   m;
    vip_expect_catcher catcher;

    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    virtual function void configure(); endfunction

    function void build_phase(uvm_phase phase);
      i2c_vif_t v;
      super.build_phase(phase);
      if (!uvm_config_db #(i2c_vif_t)::get(this, "", "vif", v)) `vip_fatal("TEST", "no vif")
      if (!uvm_config_db #(crif_t)::get(this, "", "crif", crif)) `vip_fatal("TEST", "no crif")
      catcher = new("catcher");
      mcfg  = i2c_agent_cfg::type_id::create("mcfg");  mcfg.vif = v;  mcfg.role = VIP_MASTER;
      s0cfg = i2c_agent_cfg::type_id::create("s0cfg"); s0cfg.vif = v; s0cfg.role = VIP_SLAVE; s0cfg.address = 10'h50; s0cfg.coverage_en = 0;
      s1cfg = i2c_agent_cfg::type_id::create("s1cfg"); s1cfg.vif = v; s1cfg.role = VIP_SLAVE; s1cfg.address = 10'h2A5; s1cfg.addr10 = 1; s1cfg.coverage_en = 0;
      ecfg = i2c_env_cfg::type_id::create("ecfg"); ecfg.mst_cfg = mcfg; ecfg.slv_cfg = '{s0cfg, s1cfg};
      configure();
      uvm_config_db #(i2c_env_cfg)::set(this, "env", "cfg", ecfg);
      env = i2c_env::type_id::create("env", this);
    endfunction

    function void set_speed(i2c_speed_e s);
      mcfg.speed = s; s0cfg.speed = s; s1cfg.speed = s;
    endfunction

    virtual task run_seqs(uvm_phase phase); endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      m = i2c_api_seq::type_id::create("m");
      fork m.start(env.mst.sqr); join_none
      wait (crif.rst_n === 1'b1);
      #(2us);
      run_seqs(phase);
      #(20us);
      phase.drop_objection(this);
    endtask

    function void check_phase(uvm_phase phase);
      if (env.sb != null && env.sb.n_mismatch != 0) `vip_error("I2C_TEST", "scoreboard mismatches")
      if (env.mst.mon.n_timing_err + env.mst.mon.n_frame_err != 0 && !mcfg.vif.expect_violations)
        `vip_error("I2C_TEST", "timing / framing errors reported")
    endfunction
  endclass : i2c_b2b_base_test

  class i2c_smoke_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_smoke_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      i2c_rand_seq r = i2c_rand_seq::type_id::create("r");
      if (!r.randomize() with { n == 8; }) `vip_fatal("TEST", "randomize")
      r.start(env.mst.sqr);
      if (r.n_mismatch != 0) `vip_error("I2C_TEST", "read-back mismatches")
      if (env.sb.n_match < 24) `vip_error("I2C_TEST", $sformatf("only %0d messages matched", env.sb.n_match))
    endtask
  endclass : i2c_smoke_test

  class i2c_speeds_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_speeds_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      i2c_speed_e sp[3] = '{I2C_SM, I2C_FM, I2C_FMP};
      foreach (sp[k]) begin
        i2c_rand_seq r = i2c_rand_seq::type_id::create("r");
        set_speed(sp[k]);
        #(10us);
        if (!r.randomize() with { n == 3; }) `vip_fatal("TEST", "randomize")
        r.start(env.mst.sqr);
        if (r.n_mismatch != 0) `vip_error("I2C_TEST", $sformatf("read-back mismatches at %s", sp[k].name()))
      end
    endtask
  endclass : i2c_speeds_test

  class i2c_addr10_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_addr10_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      bit [7:0] q[$];
      bit ok;
      m.addr10 = 1;
      m.write(10'h2A5, '{8'h10, 8'hA1, 8'hB2, 8'hC3}, ok);
      if (!ok) `vip_error("I2C_TEST", "10-bit write not acknowledged")
      m.write_read(10'h2A5, 8'h10, 3, q, ok);
      if (!ok || q.size() != 3 || q[0] != 8'hA1 || q[2] != 8'hC3) `vip_error("I2C_TEST", "10-bit read-back wrong")
      m.write(10'h0A5, '{8'h00}, ok);                     // another 10-bit address: nobody answers
      if (ok) `vip_error("I2C_TEST", "a 10-bit address without a slave was acknowledged")
      m.addr10 = 0;
      // the monitor must forget the refused 10-bit header: the next 7-bit message's first data byte is data, not A7:A0
      m.write(10'h50, '{8'h40, 8'h5A, 8'h6B}, ok);
      if (!ok) `vip_error("I2C_TEST", "7-bit write after a refused 10-bit header not acknowledged")
      m.write_read(10'h50, 8'h40, 2, q, ok);
      if (!ok || q.size() != 2 || q[0] != 8'h5A || q[1] != 8'h6B) `vip_error("I2C_TEST", "7-bit read-back after a refused 10-bit header wrong")
      if (s0cfg.mem.is_written(8'h11)) `vip_error("I2C_TEST", "the 7-bit slave took part in a 10-bit transfer")
    endtask
  endclass : i2c_addr10_test

  class i2c_nack_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_nack_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure(); s0cfg.nack_after = 3; endfunction
    task run_seqs(uvm_phase phase);
      bit ok;
      bit [7:0] q[$];
      m.write(10'h33, '{8'h01, 8'h02}, ok);               // no slave at 0x33: address NACK, then STOP
      if (ok) `vip_error("I2C_TEST", "address 0x33 was acknowledged")
      m.write(10'h50, '{8'h20, 8'h11, 8'h22, 8'h33, 8'h44}, ok);   // the 4th data byte is refused
      if (ok) `vip_error("I2C_TEST", "data NACK was not reported")
      s0cfg.nack_after = -1;
      m.write_read(10'h50, 8'h20, 2, q, ok);
      if (!ok || q[0] != 8'h11 || q[1] != 8'h22) `vip_error("I2C_TEST", "bytes before the NACK were not stored")
      m.read(10'h33, 2, q, ok);                            // read from an absent slave: address NACK, no data
      if (ok) `vip_error("I2C_TEST", "read address 0x33 was acknowledged")
      m.read(10'h50, 2, q, ok);                            // plain read (START, not a repeated START) from the pointer
      if (!ok || q.size() != 2) `vip_error("I2C_TEST", "plain read failed")
      if (env.mst.mon.n_nacks < 2) `vip_error("I2C_TEST", "the monitor did not see the NACKs")
    endtask
  endclass : i2c_nack_test

  class i2c_stretch_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_stretch_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure(); s0cfg.stretch_ns = 7000.0; endfunction
    task run_seqs(uvm_phase phase);
      i2c_rand_seq r = i2c_rand_seq::type_id::create("r");
      if (!r.randomize() with { n == 4; }) `vip_fatal("TEST", "randomize")
      r.start(env.mst.sqr);
      if (r.n_mismatch != 0) `vip_error("I2C_TEST", "read-back mismatches with clock stretching")
    endtask
  endclass : i2c_stretch_test

  class i2c_multi_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_multi_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      for (int k = 0; k < 6; k++) begin
        bit ok;
        bit [7:0] q[$];
        bit [7:0] p = 8'h40 + k;
        m.addr10 = k % 2;
        m.write(k % 2 ? 10'h2A5 : 10'h50, '{p, 8'(k * 3), 8'(k * 5)}, ok, 0);     // no STOP: the read starts with Sr
        m.write_read(k % 2 ? 10'h2A5 : 10'h50, p, 2, q, ok);
        if (!ok || q[0] != 8'(k * 3) || q[1] != 8'(k * 5)) `vip_error("I2C_TEST", $sformatf("multi-slave read-back %0d wrong", k))
      end
      m.addr10 = 0;
      if (env.mst.mon.n_rstart < 12) `vip_error("I2C_TEST", "repeated STARTs were not observed")
    endtask
  endclass : i2c_multi_test

  // the monitor reports a master that violates the timing of its speed mode
  class i2c_timing_test extends i2c_b2b_base_test;
    `uvm_component_utils(i2c_timing_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void configure(); ecfg.has_scoreboard = 0; endfunction
    task run_seqs(uvm_phase phase);
      bit ok;
      int unsigned t0;
      mcfg.vif.expect_violations = 1;
      catcher.add_expect("I2C_T_LOW", UVM_ERROR, 1);
      catcher.add_expect("I2C_T_HIGH", UVM_ERROR, 1);
      catcher.install();
      mcfg.t_low_ns = 1000.0;  m.write(10'h50, '{8'h01, 8'h02}, ok);      // Fm needs tLOW >= 1300 ns
      mcfg.t_low_ns = 0.0;     mcfg.t_high_ns = 400.0;
      m.write(10'h50, '{8'h03, 8'h04}, ok);                               // Fm needs tHIGH >= 600 ns
      mcfg.t_high_ns = 0.0;
      t0 = env.mst.mon.n_timing_err;
      m.write(10'h50, '{8'h05, 8'h06}, ok);
      if (env.mst.mon.n_timing_err != t0) `vip_error("I2C_TEST", "timing errors reported for a compliant master")
    endtask
    function void check_phase(uvm_phase phase); void'(catcher.check_all()); endfunction
  endclass : i2c_timing_test

  // ================================================================ I2C IP of the repository (i2c_dut_tb_top)
  class i2c_dut_base_test extends uvm_test;
    `uvm_component_utils(i2c_dut_base_test)
    i2c_env       ienv;
    i2c_env_cfg   iecfg;
    i2c_agent_cfg scfg;
    apb_env       aenv;
    apb_env_cfg   aecfg;
    apb_agent_cfg apcfg;
    crif_t        crif;
    apb_api_seq   regs;

    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    function void build_phase(uvm_phase phase);
      i2c_vif_t iv;
      apb_vif_t av;
      super.build_phase(phase);
      if (!uvm_config_db #(i2c_vif_t)::get(this, "", "vif", iv)) `vip_fatal("TEST", "no i2c vif")
      if (!uvm_config_db #(apb_vif_t)::get(this, "", "apb_vif", av)) `vip_fatal("TEST", "no apb vif")
      if (!uvm_config_db #(crif_t)::get(this, "", "crif", crif)) `vip_fatal("TEST", "no crif")
      scfg = i2c_agent_cfg::type_id::create("scfg"); scfg.vif = iv; scfg.role = VIP_SLAVE; scfg.address = 10'h50;
      iecfg = i2c_env_cfg::type_id::create("iecfg"); iecfg.slv_cfg = '{scfg};
      apcfg = apb_agent_cfg::type_id::create("apcfg"); apcfg.vif = av; apcfg.role = VIP_MASTER;
      aecfg = apb_env_cfg::type_id::create("aecfg"); aecfg.mst_cfg = apcfg; aecfg.has_scoreboard = 0;
      uvm_config_db #(i2c_env_cfg)::set(this, "ienv", "cfg", iecfg);
      uvm_config_db #(apb_env_cfg)::set(this, "aenv", "cfg", aecfg);
      ienv = i2c_env::type_id::create("ienv", this);
      aenv = apb_env::type_id::create("aenv", this);
    endfunction

    task wr(bit [11:0] a, bit [31:0] d); bit e; regs.write(a, d, e); if (e) `vip_error("I2C_TEST", $sformatf("PSLVERR at 0x%0h", a)) endtask
    task rd(bit [11:0] a, output bit [31:0] d); bit e; regs.read(a, d, e); if (e) `vip_error("I2C_TEST", $sformatf("PSLVERR at 0x%0h", a)) endtask

    // CMD: data[7:0] START[8] STOP[9] RD[10] NACK[11] WR[12]
    task cmd(bit [7:0] d, bit start = 0, bit stop = 0, bit r = 0, bit nack = 0, bit w = 0);
      bit [31:0] st;
      do rd('h014, st); while (st[0]);                                       // CMD_FULL
      wr('h00C, d | (start << 8) | (stop << 9) | (r << 10) | (nack << 11) | (w << 12));
    endtask

    task wait_done();
      bit [31:0] st;
      int unsigned g = 0;
      do begin rd('h014, st); g++; end while ((st[4] || !st[1]) && g < 200000);
    endtask

    task setup(i2c_speed_e s);
      scfg.speed = s;
      wr('h008, (s == I2C_SM) ? 124 : 31);                                   // 100 kHz / 400 kHz at 50 MHz
      wr('h004, 32'h3);                                                       // EN | NACK_STOP
    endtask

    task ip_write(bit [6:0] a, bit [7:0] bytes[$]);
      cmd({a, 1'b0}, 1, 0, 0, 0, 1);
      foreach (bytes[i]) cmd(bytes[i], 0, i == bytes.size() - 1, 0, 0, 1);
      wait_done();
    endtask

    task ip_read(bit [6:0] a, bit [7:0] ptr, int unsigned n, output bit [7:0] got[$]);
      cmd({a, 1'b0}, 1, 0, 0, 0, 1);
      cmd(ptr, 0, 0, 0, 0, 1);
      cmd({a, 1'b1}, 1, 0, 0, 0, 1);
      for (int i = 0; i < n; i++) cmd(0, 0, i == n - 1, 1, i == n - 1, 0);
      wait_done();
      got.delete();
      for (int i = 0; i < n; i++) begin bit [31:0] v; rd('h010, v); got.push_back(v[7:0]); if (!v[31]) `vip_error("I2C_TEST", "RX FIFO empty") end
    endtask

    virtual task run_seqs(uvm_phase phase); endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      regs = apb_api_seq::type_id::create("regs");
      fork regs.start(aenv.mst.sqr); join_none
      wait (crif.rst_n === 1'b1);
      repeat (5) @(posedge crif.clk);
      run_seqs(phase);
      #(30us);
      phase.drop_objection(this);
    endtask

    function void check_phase(uvm_phase phase);
      if (ienv.sb.n_mismatch != 0) `vip_error("I2C_TEST", "scoreboard mismatches")
      if (ienv.slv[0].mon.n_timing_err + ienv.slv[0].mon.n_frame_err != 0) `vip_error("I2C_TEST", "the IP violates I2C timing / framing")
    endfunction
  endclass : i2c_dut_base_test

  class i2c_dut_wr_rd_test extends i2c_dut_base_test;
    `uvm_component_utils(i2c_dut_wr_rd_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      bit [7:0] got[$];
      setup(I2C_FM);
      ienv.sb.expect_msg(10'h50, 0, '{8'h30, 8'hDE, 8'hAD, 8'hBE, 8'hEF});
      ip_write(7'h50, '{8'h30, 8'hDE, 8'hAD, 8'hBE, 8'hEF});
      ienv.sb.expect_msg(10'h50, 0, '{8'h30});
      ienv.sb.expect_msg(10'h50, 1, '{8'hDE, 8'hAD, 8'hBE, 8'hEF});
      ip_read(7'h50, 8'h30, 4, got);
      if (got != '{8'hDE, 8'hAD, 8'hBE, 8'hEF}) `vip_error("I2C_TEST", "IP read back wrong data")
      if (scfg.mem.read_byte(8'h31) != 8'hAD) `vip_error("I2C_TEST", "slave memory not written")
    endtask
  endclass : i2c_dut_wr_rd_test

  class i2c_dut_nack_test extends i2c_dut_base_test;
    `uvm_component_utils(i2c_dut_nack_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      bit [31:0] v;
      setup(I2C_FM);
      ienv.sb.expect_msg(10'h12, 0, '{}, 0);                                  // address NACK
      ip_write(7'h12, '{8'h00, 8'h01});
      rd('h018, v);
      if (!v[2]) `vip_error("I2C_TEST", "the IP did not flag the NACK")
    endtask
  endclass : i2c_dut_nack_test

  class i2c_dut_stretch_test extends i2c_dut_base_test;
    `uvm_component_utils(i2c_dut_stretch_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      bit [7:0] got[$];
      setup(I2C_FM);
      scfg.stretch_ns = 15000.0;
      ienv.sb.expect_msg(10'h50, 0, '{8'h08, 8'h5A, 8'hA5});
      ip_write(7'h50, '{8'h08, 8'h5A, 8'hA5});
      ienv.sb.expect_msg(10'h50, 0, '{8'h08});
      ienv.sb.expect_msg(10'h50, 1, '{8'h5A, 8'hA5});
      ip_read(7'h50, 8'h08, 2, got);
      if (got != '{8'h5A, 8'hA5}) `vip_error("I2C_TEST", "data wrong with clock stretching")
    endtask
  endclass : i2c_dut_stretch_test

  class i2c_dut_timing_test extends i2c_dut_base_test;
    `uvm_component_utils(i2c_dut_timing_test)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    task run_seqs(uvm_phase phase);
      i2c_speed_e sp[2] = '{I2C_SM, I2C_FM};
      foreach (sp[k]) begin
        bit [7:0] got[$];
        setup(sp[k]);
        ienv.sb.expect_msg(10'h50, 0, '{8'h70, 8'h01, 8'h02});
        ip_write(7'h50, '{8'h70, 8'h01, 8'h02});
        ienv.sb.expect_msg(10'h50, 0, '{8'h70});
        ienv.sb.expect_msg(10'h50, 1, '{8'h01, 8'h02});
        ip_read(7'h50, 8'h70, 2, got);
        #(20us);
      end
    endtask
  endclass : i2c_dut_timing_test

endpackage : i2c_test_pkg

`endif
