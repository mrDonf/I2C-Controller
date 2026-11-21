// -----------------------------------------------------------------------------
// Layer 1 - configuration sequences (spec 5.2, 6.1, 6.2, 6.6)
//   i2c_ip_init_vseq         initialisation of 6.2 (speed mode, roles, target addresses, TCFG, INTR_EN, IDLE_VAL)
//   i2c_ip_hs_cfg_vseq       Hs-mode timing + controller code + HS_EN (6.6)
//   i2c_ip_timeout_cfg_vseq  SCL low timeout (5.2.18)
//   i2c_ip_fifo_thr_vseq     FIFO thresholds (5.2.4)
//   i2c_ip_ext_dev_cfg_vseq  behaviour of the external devices (VIP slaves: clock stretching, data NACK)
// All fields are rand with soft defaults: a test either starts them as they are or randomizes them with inline
// constraints.
// -----------------------------------------------------------------------------

class i2c_ip_init_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_init_vseq)

  rand i2c_ip_speed_e speed = SPD_FM;
  rand bit            ctrl_en = 1, tgt_en = 1;
  rand bit [3:0]      tcfg;           // {DEVID_EN, TNACK, NOSTRETCH, GC_EN}
  rand bit [21:0]     intr_en;
  rand bit [6:0]      addr0 = 7'h3A, addr1 = 7'h2B;   // TADDR.ADDR0 / ADDR1
  rand bit [15:0]     idle_val = 1000;      // BUSIDLE.IDLE_VAL
  bit                 wait_idle = 1;

  // addresses the tests use for other purposes: reserved (4.5.1), VIP slaves, absent devices, foreign address
  constraint c_addr {
    addr0 != addr1;
    !(addr0 inside {[7'h00:7'h07], [7'h78:7'h7F], 7'h50, 7'h51, [7'h20:7'h27], 7'h44, 7'h55});
    !(addr1 inside {[7'h00:7'h07], [7'h78:7'h7F], 7'h50, 7'h51, [7'h20:7'h27], 7'h44, 7'h55});
  }
  constraint c_idle { idle_val inside {[100:2000]}; }
  constraint c_default {
    soft speed == SPD_FM;  soft ctrl_en == 1;  soft tgt_en == 1;  soft tcfg == 0;  soft intr_en == 0;
    soft addr0 == 7'h3A;   soft addr1 == 7'h2B; soft idle_val == 1000;
  }

  function new(string name = "i2c_ip_init_vseq"); super.new(name); endfunction

  task main();
    `uvm_info("INIT", $sformatf("%s CTRL_EN=%0d TGT_EN=%0d TCFG=0x%0h INTR_EN=0x%06h ADDR0=0x%02h ADDR1=0x%02h IDLE_VAL=%0d",
              speed.name(), ctrl_en, tgt_en, tcfg, intr_en, addr0, addr1, idle_val), UVM_MEDIUM)
    own0 = addr0; own1 = addr1;
    init_ip(speed, ctrl_en, tgt_en, 32'(tcfg), 32'(intr_en), wait_idle, idle_val);
  endtask
endclass : i2c_ip_init_vseq

// ---------------------------------------------------------------- Hs-mode (6.6): spec 6.2 Hs timing, controller code
class i2c_ip_hs_cfg_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_hs_cfg_vseq)

  rand bit [2:0] mcode = 3;          // controller code 0000_1xxx (HSCFG.MCODE)
  bit            hs_en = 1;
  constraint c_default { soft mcode == 3; }

  function new(string name = "i2c_ip_hs_cfg_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] c;
    // the VIP has no Hs-mode: its slaves answer with a short SDA hold and its monitors skip the F/S timing limits
    cfg.i2c_mst_cfg.check_timing = 0;
    foreach (cfg.i2c_slv_cfg[i]) begin cfg.i2c_slv_cfg[i].check_timing = 0; cfg.i2c_slv_cfg[i].slave_hd_dat_ns = 20; end
    rd(R_CTRL, c);
    wr(R_CTRL, 32'h0);
    reg_wr(ral.HS_TIM_SCL, {16'(HS_H), 16'(HS_L)});
    reg_wr(ral.HS_TIM_STA, {16'(HS_STA), 16'(HS_STA)});
    reg_wr(ral.HS_TIM_STO, 16'(HS_STA));
    reg_wr(ral.HS_TIM_DAT, {16'(HS_SUDAT), 16'(HS_HDDAT)});
    reg_wr(ral.HSCFG, mcode);
    wr(R_CTRL, (c & 32'h14) | 32'h3 | (hs_en << 3));         // EN, CTRL_EN, HS_EN; TGT_EN / STARTBYTE_EN kept
    wait_dut_idle();
  endtask
endclass : i2c_ip_hs_cfg_vseq

// ---------------------------------------------------------------- SCL low timeout (5.2.18)
class i2c_ip_timeout_cfg_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_timeout_cfg_vseq)

  rand bit        tout_en = 1;
  rand bit [23:0] tout_val = 2000;          // PCLK cycles
  constraint c_default { soft tout_en == 1; soft tout_val == 2000; tout_val > 0; }

  function new(string name = "i2c_ip_timeout_cfg_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] c;
    rd(R_CTRL, c);
    wr(R_CTRL, 32'h0);
    reg_wr(ral.TIMEOUT, {tout_en, 7'h0, tout_val});
    wr(R_CTRL, c & 32'h1F);
    if (c[0]) wait_dut_idle();
  endtask
endclass : i2c_ip_timeout_cfg_vseq

// ---------------------------------------------------------------- FIFO thresholds (5.2.4)
class i2c_ip_fifo_thr_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_fifo_thr_vseq)

  rand bit [5:0] cmd_thr, crx_thr, ttx_thr, trx_thr;
  constraint c_default { soft cmd_thr == 0; soft crx_thr == 0; soft ttx_thr == 0; soft trx_thr == 0; }

  function new(string name = "i2c_ip_fifo_thr_vseq"); super.new(name); endfunction

  task main();
    reg_wr(ral.FIFO_CTRL, {2'b0, trx_thr, 2'b0, ttx_thr, 2'b0, crx_thr, 2'b0, cmd_thr});
  endtask
endclass : i2c_ip_fifo_thr_vseq

// ---------------------------------------------------------------- external devices (VIP slaves)
class i2c_ip_ext_dev_cfg_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_ext_dev_cfg_vseq)

  int       slv = -1;                // VIP slave index, -1 = all
  rand int  stretch_ns = 0;              // SCL held low after every acknowledge (target clock stretching, F7)
  rand int  nack_after = -1;              // NACK the n-th data byte of a write (-1 = never)
  constraint c_default { soft stretch_ns == 0; soft nack_after == -1; stretch_ns inside {[0:20000]}; nack_after >= -1; }

  function new(string name = "i2c_ip_ext_dev_cfg_vseq"); super.new(name); endfunction

  task main();
    foreach (cfg.i2c_slv_cfg[i]) if (slv < 0 || slv == i) begin
      cfg.i2c_slv_cfg[i].stretch_ns = stretch_ns;
      cfg.i2c_slv_cfg[i].nack_after = nack_after;
    end
  endtask
endclass : i2c_ip_ext_dev_cfg_vseq
