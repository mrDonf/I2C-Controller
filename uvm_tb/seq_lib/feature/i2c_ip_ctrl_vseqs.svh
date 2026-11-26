// -----------------------------------------------------------------------------
// Layer 2 - controller feature sequences (spec 4.4, 6.3 - 6.8, 6.12, 6.13)
//   The data path (CMD -> bus -> CRX) is checked by the scoreboard on every transfer; these sequences check the
//   software-visible behaviour (events, CSTAT, STATUS, FIFO levels) and the external device state (VIP slave memory).
//   They expect the controller to be enabled (CTRL = EN | CTRL_EN) by the test.
// -----------------------------------------------------------------------------

// ---------------------------------------------------------------- VP-CTL-04: completion interrupt (6.12)
class i2c_ctrl_cmpl_irq_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_cmpl_irq_vseq)
  function new(string name = "i2c_ctrl_cmpl_irq_vseq"); super.new(name); endfunction

  task main();
    bit [31:0] v, en0;
    rd(R_INTR_EN, en0);
    wr(R_INTR_EN, en0 | 32'h4);
    push_cmds('{c_start(7'h51, 0), c_wr(8'h00, 1)});
    wait (side.irq_ctrl === 1'b1);
    rd(R_INTR_STAT, v);
    chk(v[I_CMPL], "irq_ctrl_o raised by CMPL");
    w1c(v & EVENT_MASK);
    wait_cycles(2);
    chk(side.irq_ctrl === 1'b0, "irq_ctrl_o released 1 PCLK after the W1C");
    chk_cstat(0);
    wr(R_INTR_EN, en0);
  endtask
endclass : i2c_ctrl_cmpl_irq_vseq

// ---------------------------------------------------------------- controller idle state after traffic
class i2c_ctrl_idle_chk_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_idle_chk_vseq)
  function new(string name = "i2c_ctrl_idle_chk_vseq"); super.new(name); endfunction

  task main();
    chk_rd(R_STATUS, 32'h0000_AA00, 32'h0000_FFFE, "STATUS idle (no transfer, FIFOs empty)");
    chk_rd(R_FIFO_STAT, 0, '1, "FIFO levels 0");
    chk_ev(I_NACK, 0, "no NACK:");
    chk_ev(I_CMD_ERR, 0, "no CMD_ERR:");
    chk_ev(I_ARB_LOST, 0, "no ARB_LOST:");
    chk_rd(R_CSTAT, 0, '1, "CSTAT clean");
  endtask
endclass : i2c_ctrl_idle_chk_vseq

// ---------------------------------------------------------------- VP-CTL-05: bus timing vs spec 4.4.2 / 6.2
//   checks the results of an i2c_ip_timing_meas_vseq run in parallel with controller traffic
class i2c_ctrl_timing_chk_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_timing_chk_vseq)
  function new(string name = "i2c_ctrl_timing_chk_vseq"); super.new(name); endfunction

  i2c_ip_timing_meas_vseq meas;

  function void chk_time(string what, realtime got, real exp_ns, real tol_ns);
    chk(got >= (exp_ns - tol_ns) * 1ns && got <= (exp_ns + tol_ns) * 1ns,
        $sformatf("%s = %0.1f ns, spec %0.1f ns (+/- %0.1f)", what, got / 1ns, exp_ns, tol_ns));
  endfunction

  function void chk_min(string what, realtime got, real min_ns);
    chk(got >= min_ns * 1ns, $sformatf("%s = %0.1f ns >= %0.1f ns", what, got / 1ns, min_ns));
  endfunction

  task main();
    int unsigned lat = tm.spklen + 3;
    i2c_timing_t um = i2c_spec(vip_speed(spd));
    string m = spd.name();
    `uvm_info("TIMING", $sformatf("%s: tLOW min %0.0f mode %0.0f | tHIGH min %0.0f mode %0.0f | tHD;DAT %0.0f | tHD;STA %0.0f | tSU;STA %0.0f | tSU;STO %0.0f | tBUF %0.0f ns",
              m, meas.qmin(meas.m_low) / 1ns, meas.qmode(meas.m_low) / 1ns, meas.qmin(meas.m_high) / 1ns, meas.qmode(meas.m_high) / 1ns,
              meas.qmin(meas.m_hddat) / 1ns, meas.qmin(meas.m_hdsta) / 1ns, meas.qmin(meas.m_susta) / 1ns, meas.qmin(meas.m_susto) / 1ns,
              meas.qmin(meas.m_buf) / 1ns), UVM_LOW)
    chk(meas.m_low.size() > 0 && meas.m_susta.size() > 0 && meas.m_buf.size() > 0, "timing samples recorded (incl. Sr and bus free)");
    // exact formulas of 4.4.2 with t_r = t_f = 0
    chk_time($sformatf("%s tLOW = (SCL_LOW + LAT) T", m),   meas.qmode(meas.m_low),  (tm.scl_low  + lat) * T, T);
    chk_time($sformatf("%s tHIGH = (SCL_HIGH + LAT) T", m), meas.qmode(meas.m_high), (tm.scl_high + lat) * T, T);
    chk_time($sformatf("%s tLOW min", m),  meas.qmin(meas.m_low),  (tm.scl_low  + lat) * T, T);
    chk_time($sformatf("%s tHIGH min", m), meas.qmin(meas.m_high), (tm.scl_high + lat) * T, T);
    chk_time($sformatf("%s tHD;DAT = (LAT + THD_DAT + 1) T", m), meas.qmin(meas.m_hddat), (lat + tm.thd_dat + 1) * T, T);
    // register-to-bus mapping of 4.11.2 (counter starts at the observed edge) and the UM10204 minimums
    chk_min($sformatf("%s tHD;STA", m), meas.qmin(meas.m_hdsta), tm.thd_sta * T);
    chk_min($sformatf("%s tSU;STA", m), meas.qmin(meas.m_susta), tm.tsu_sta * T);
    chk_min($sformatf("%s tSU;STO", m), meas.qmin(meas.m_susto), tm.tsu_sto * T);
    chk_min($sformatf("%s tBUF", m),    meas.qmin(meas.m_buf),   tm.tbuf * T);
    chk_min($sformatf("%s UM10204 tLOW", m),  meas.qmin(meas.m_low),  um.t_low);
    chk_min($sformatf("%s UM10204 tHIGH", m), meas.qmin(meas.m_high), um.t_high);
    chk_min($sformatf("%s UM10204 tHD;STA", m), meas.qmin(meas.m_hdsta), um.t_hd_sta);
    chk_min($sformatf("%s UM10204 tSU;STA", m), meas.qmin(meas.m_susta), um.t_su_sta);
    chk_min($sformatf("%s UM10204 tSU;STO", m), meas.qmin(meas.m_susto), um.t_su_sto);
    chk_min($sformatf("%s UM10204 tBUF", m),    meas.qmin(meas.m_buf),   um.t_buf);
    chk_min($sformatf("%s f_SCL period", m), meas.qmode(meas.m_low) + meas.qmode(meas.m_high),
            (spd == SPD_SM) ? 10000.0 : (spd == SPD_FM) ? 2500.0 : 1000.0);
  endtask
endclass : i2c_ctrl_timing_chk_vseq

// ---------------------------------------------------------------- VP-CTL-06: CMD starvation, CMD_THR refill (6.3)
class i2c_ctrl_cmd_stall_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_cmd_stall_vseq)
  function new(string name = "i2c_ctrl_cmd_stall_vseq"); super.new(name); endfunction

  rand int unsigned n = 0;             // data bytes after the pointer (0: 3 x FIFO_DEPTH + 3)
  rand bit [7:0]    ptr = 8'h20;
  rand int unsigned thr = 4;              // CMD_THR used for the refill interrupt
  constraint c_def { soft n == 0; soft ptr == 8'h20; soft thr == 4; n <= 128; }

  task main();
    bit [7:0]  d[$];
    bit [12:0] c[$];
    bit [31:0] v;
    int unsigned k;
    if (n == 0) n = 3 * depth + 3;
    if (thr >= depth) thr = depth / 2;
    rnd_bytes(n, d);
    d.push_front(ptr);
    c.push_back(c_start(7'h50, 0));
    foreach (d[i]) c.push_back(c_wr(d[i], i == d.size() - 1));
    for (k = 0; k < 4; k++) wr(R_CMD, c[k]);
    wait_ev(I_CMD_STALL, 2ms, 0, "controller starved");
    chk_rd(R_STATUS, 32'h0000_0007, 32'h0000_0007, "STATUS: BUS_BUSY, CTRL_ACTIVE, CTRL_HOLD while stalled");
    #(50us);
    chk(bus.scl === 1'b0, "SCL held low while the CMD FIFO is empty");
    w1c(32'h1 << I_CMD_STALL);
    wr(R_FIFO_CTRL, thr);
    wr(R_INTR_EN, 32'h1);                                    // CMD_THR level interrupt
    while (k < c.size()) begin
      wait (side.irq_ctrl === 1'b1);                         // "ISR"
      rd(R_STATUS, v);
      while (!v[8] && k < c.size()) begin                    // fill until CMD_FULL
        wr(R_CMD, c[k++]);
        rd(R_STATUS, v);
      end
      if (k == c.size()) wr(R_INTR_EN, 0);                   // no more data: CMD_THR off (6.3)
    end
    wait_cmpl(0, "long write");
    for (int i = 1; i < d.size(); i++)
      chk(slv_mem(0, ptr + i - 1) == d[i], $sformatf("VIP slave mem[0x%02h]", 8'(ptr + i - 1)));
    wr(R_FIFO_CTRL, 0);
  endtask
endclass : i2c_ctrl_cmd_stall_vseq

// ---------------------------------------------------------------- VP-CTL-07: CRX full -> controller holds SCL (4.4.3)
class i2c_ctrl_crx_full_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_crx_full_vseq)
  function new(string name = "i2c_ctrl_crx_full_vseq"); super.new(name); endfunction

  rand int unsigned nrd = 0;           // 0: 2 x FIFO_DEPTH + 5
  rand bit [7:0]    ptr = 8'h20;
  constraint c_def { soft nrd == 0; soft ptr == 8'h20; nrd <= 128; }

  task main();
    bit [12:0] c[$];
    bit [31:0] v;
    bit [8:0]  r[$];
    if (nrd == 0) nrd = 2 * depth + 5;
    p_sequencer.side.clear_seen();
    c.push_back(c_start(7'h50, 0)); c.push_back(c_wr(ptr)); c.push_back(c_start(7'h50, 1));
    for (int i = 0; i < nrd; i++) c.push_back(c_rd(0, i == nrd - 1));
    fork
      push_cmds(c);
      begin
        wait_lvl(1, depth);
        chk_rd(R_STATUS, 32'h0000_0400, 32'h0000_0400, "STATUS.CRX_FULL");
        #(60us);                                             // software late: the controller must wait
        chk_rd(R_FIFO_STAT, depth << 8, 32'h3F << 8, "CRX level stays FIFO_DEPTH (no overflow)");
        chk(p_sequencer.side.seen_ctrl_hold, "controller held SCL while CRX was full (CTRL_HOLD)");
        wr(R_FIFO_CTRL, (depth / 2) << 8);                   // CRX_THR = depth/2
        wr(R_INTR_EN, 32'h2);
        while (r.size() < nrd) begin
          fork begin
            fork
              wait (side.irq_ctrl === 1'b1);
              wait (side.d_intr_stat[I_CMPL] === 1'b1);
            join_any
            disable fork;
          end join
          rd(R_FIFO_STAT, v);
          if (v[13:8] == 0 && side.d_intr_stat[I_CMPL] !== 1'b1) continue;
          repeat (v[13:8]) begin rd(R_CRXDATA, v); r.push_back(v[8:0]); end
        end
        wr(R_INTR_EN, 0);
      end
    join
    wait_cmpl(0, "long read");
    foreach (r[i]) chk(r[i][7:0] == slv_mem(0, ptr + i) && r[i][8] == (i == nrd - 1), $sformatf("long read byte %0d = %03h", i, r[i]));
    wr(R_FIFO_CTRL, 0);
  endtask
endclass : i2c_ctrl_crx_full_vseq

// ---------------------------------------------------------------- VP-CTL-08: address NACK (4.4.4, 6.13)
class i2c_ctrl_addr_nack_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_addr_nack_vseq)
  function new(string name = "i2c_ctrl_addr_nack_vseq"); super.new(name); endfunction

  rand bit [6:0] absent = 7'h23;       // address nobody answers
  rand bit       rd_dir;
  constraint c_def { absent inside {[7'h20:7'h27]}; }

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    // the queued commands behind the NACKed address (incl. a second transfer) are flushed
    if (rd_dir) cs.cmds = '{c_start(absent, 1), c_rd(), c_rd(0, 1), c_start(7'h50, 0), c_wr(8'h00, 1)};
    else        cs.cmds = '{c_start(absent, 0), c_wr(8'h01), c_wr(8'h02, 1), c_start(7'h50, 0), c_wr(8'h00, 1)};
    cs.exp = CMD_EXP_NACK; cs.exp_cstat = 7'h01; cs.n_crx = 0;
    cs.what = $sformatf("address NACK 0x%02h %s", absent, rd_dir ? "R" : "W");
    run_seq(cs);
  endtask
endclass : i2c_ctrl_addr_nack_vseq

// ---------------------------------------------------------------- VP-CTL-08: own target never answers own controller (R4)
class i2c_ctrl_own_tgt_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_own_tgt_vseq)
  function new(string name = "i2c_ctrl_own_tgt_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    cs.cmds = '{c_start(own0, 0), c_wr(8'h01, 1)};
    cs.exp = CMD_EXP_NACK; cs.exp_cstat = 7'h01; cs.what = "own target address";
    run_seq(cs);
    chk_rd(R_FIFO_STAT, 0, '1, "no TRX entry from the own controller");
  endtask
endclass : i2c_ctrl_own_tgt_vseq

// ---------------------------------------------------------------- VP-CTL-08: irq_ctrl_o for NACK (6.12.4)
class i2c_ctrl_nack_irq_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_nack_irq_vseq)
  function new(string name = "i2c_ctrl_nack_irq_vseq"); super.new(name); endfunction

  task main();
    wr(R_INTR_EN, 32'h0000_003C);
    push_cmds('{c_start(7'h26, 0), c_wr(8'h01, 1)});
    wait (side.irq_ctrl === 1'b1);
    wait_ev(I_NACK, 1ms, 1);
    wait_cmpl(7'h01, "irq NACK");
    wait_cycles(2);
    chk(side.irq_ctrl === 1'b0, "irq_ctrl_o released after NACK + CMPL W1C");
    wr(R_INTR_EN, 0);
  endtask
endclass : i2c_ctrl_nack_irq_vseq

// ---------------------------------------------------------------- VP-CTL-09: data NACK (4.4.4)
//   the VIP slave refuses data byte `nack_after` (configured by the test through i2c_ip_ext_dev_cfg_vseq)
class i2c_ctrl_data_nack_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_data_nack_vseq)
  function new(string name = "i2c_ctrl_data_nack_vseq"); super.new(name); endfunction

  int unsigned nack_after = 2;         // must match the VIP slave configuration
  int unsigned n_data     = 5;         // bytes queued (including the pointer)

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    cs.cmds.push_back(c_start(7'h51, 0));
    for (int i = 0; i < n_data; i++) cs.cmds.push_back(c_wr(8'h30 + i, i == n_data - 1));
    cs.exp = CMD_EXP_NACK; cs.exp_cstat = 7'h02; cs.what = $sformatf("data NACK of byte %0d", nack_after);
    run_seq(cs);
    chk(p_sequencer.sb.last_ctrl_msg.data.size() == nack_after + 1,
        $sformatf("transfer stopped after the NACKed byte (%0d bytes on the bus)", p_sequencer.sb.last_ctrl_msg.data.size()));
  endtask
endclass : i2c_ctrl_data_nack_vseq

// ---------------------------------------------------------------- VP-CTL-10: NAKOK (4.4.4, 6.8)
class i2c_ctrl_nakok_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_nakok_vseq)
  function new(string name = "i2c_ctrl_nakok_vseq"); super.new(name); endfunction

  bit addr_case = 0;                   // 1: NAKOK on the address of an absent device, 0: NAKOK on refused data

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    if (addr_case) cs.cmds = '{c_start(7'h25, 0, 0, 1), c_wr(8'hAB, 1, 1)};
    else           cs.cmds = '{c_start(7'h51, 0), c_wr(8'h40, 0, 1), c_wr(8'h41, 0, 1), c_wr(8'h42, 0, 1), c_wr(8'h43, 0, 1), c_wr(8'h44, 1, 1)};
    cs.exp = CMD_EXP_CMPL; cs.what = addr_case ? "NAKOK address" : "NAKOK data";
    run_seq(cs);
    chk_ev(I_NACK, 0, {cs.what, ": no NACK event"});
    if (!addr_case) chk(p_sequencer.sb.last_ctrl_msg.data.size() == 5, "NAKOK: all bytes sent");
  endtask
endclass : i2c_ctrl_nakok_vseq

// ---------------------------------------------------------------- VP-CTL-11: command errors E1 - E5 (4.4.5)
typedef enum { E1_DATA_NO_START, E5_START_READ_IDLE, E2_READ_IN_WRITE, E2_WRITE_IN_READ, E3_START_AFTER_ACK,
               E4_NO_START_AFTER_NACK, E5_START_READ_IN_XFER } i2c_ip_cmderr_e;

class i2c_ctrl_cmderr_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_cmderr_vseq)
  function new(string name = "i2c_ctrl_cmderr_vseq"); super.new(name); endfunction

  rand i2c_ip_cmderr_e kind;

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    cs.what = kind.name();
    cs.exp  = CMD_EXP_ERR;
    case (kind)
      E1_DATA_NO_START:       begin cs.cmds = '{c_wr(8'h55), c_wr(8'h66, 1)};                                   cs.exp = CMD_EXP_ERR_IDLE; cs.n_crx = 0; end
      E5_START_READ_IDLE:     begin cs.cmds = '{13'h0500 | {7'h50, 1'b1}};                                     cs.exp = CMD_EXP_ERR_IDLE; cs.n_crx = 0; end
      E2_READ_IN_WRITE:       begin cs.cmds = '{c_start(7'h50, 0), c_rd(0, 1)};                                cs.n_crx = 0; end
      E2_WRITE_IN_READ:       begin cs.cmds = '{c_start(7'h50, 1), c_rd(), c_wr(8'h11, 1)};                    cs.n_crx = 1; end
      E3_START_AFTER_ACK:     begin cs.cmds = '{c_start(7'h50, 1), c_rd(), c_start(7'h50, 0), c_wr(8'h00, 1)}; cs.n_crx = 1; end
      E4_NO_START_AFTER_NACK: begin cs.cmds = '{c_start(7'h50, 1), c_rd(1, 0), c_rd(0, 1)};                    cs.n_crx = 1; end
      E5_START_READ_IN_XFER:  begin cs.cmds = '{c_start(7'h50, 0), c_wr(8'h00), 13'h0500 | {7'h50, 1'b1}};     cs.n_crx = 0; end
    endcase
    run_seq(cs);
  endtask
endclass : i2c_ctrl_cmderr_vseq

// legal: READ + NACK followed by Sr (6.5 variant), LAST on both NACKed bytes
class i2c_ctrl_nack_rstart_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_nack_rstart_vseq)
  function new(string name = "i2c_ctrl_nack_rstart_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_ctrl_cmd_vseq cs = i2c_ip_ctrl_cmd_vseq::type_id::create("cs");
    cs.cmds = '{c_start(7'h50, 1), c_rd(1, 0), c_start(7'h51, 1), c_rd(0, 1)};
    cs.n_crx = 2; cs.what = "READ+NACK then Sr";
    run_seq(cs);
    chk(cs.crx.size() == 2 && cs.crx[0][8] && cs.crx[1][8], "LAST set on both NACKed bytes");
  endtask
endclass : i2c_ctrl_nack_rstart_vseq

// ---------------------------------------------------------------- VP-CTL-12: CTRL.ABORT (4.4.9, 6.13)
typedef enum { AB_IDLE, AB_WRITE_STALL, AB_READ_STALL, AB_READ_INFLIGHT, AB_WAITBUS, AB_READ_ADDR } i2c_ip_abort_e;

class i2c_ctrl_abort_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_abort_vseq)
  function new(string name = "i2c_ctrl_abort_vseq"); super.new(name); endfunction

  rand i2c_ip_abort_e kind;

  task main();
    int unsigned s0;
    case (kind)
      AB_IDLE: begin                                         // not owning the bus: flush + USER_ABORT, no bus activity
        wr(R_CTRL, 32'h1);                                   // controller disabled: commands stay queued
        push_cmds('{c_start(7'h50, 0), c_wr(8'h01), c_wr(8'h02, 1)});
        s0 = p_sequencer.side.n_start;
        wr(R_CTRL, 32'h0001_0001);
        wait_cycles(4);
        chk_rd(R_CTRL, 32'h0000_0001, 32'h0003_001F, "CTRL.ABORT self-cleared");
        chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed by ABORT");
        chk_cstat(7'h08, "idle ABORT");
        chk(p_sequencer.side.n_start == s0, "idle ABORT: no bus activity");
        chk_ev(I_CMPL, 0, "idle ABORT: no CMPL");
        wr(R_CTRL, 32'h3);
        wait_dut_idle();
      end
      AB_WRITE_STALL: begin                                  // current byte done, STOP, CMPL, USER_ABORT
        push_cmds('{c_start(7'h50, 0), c_wr(8'h60), c_wr(8'hAA)});
        wait_ev(I_CMD_STALL, 2ms, 1, "write stall");
        wr(R_CTRL, 32'h0001_0003);
        chk_rd(R_CTRL, 32'h0001_0003, 32'h0001_0003, "CTRL.ABORT reads 1 while the abort is in progress");
        wait_cmpl(7'h08, "abort write");
        chk_rd(R_CTRL, 32'h0000_0003, 32'h0001_0003, "CTRL.ABORT cleared after the abort");
        chk(p_sequencer.sb.last_ctrl_msg.stop && p_sequencer.sb.last_ctrl_msg.data.size() == 2, "abort write: 2 bytes then STOP");
      end
      AB_READ_STALL: begin                                   // dummy read with NACK + STOP; CRX keeps the real bytes
        push_cmds('{c_start(7'h50, 1), c_rd(), c_rd()});
        wait_ev(I_CMD_STALL, 2ms, 1, "read stall");
        wr(R_CTRL, 32'h0001_0003);
        wait_cmpl(7'h08, "abort read");
        begin
          i2c_item m = p_sequencer.sb.last_ctrl_msg;
          chk(m.read && m.stop && m.data.size() == 3 && m.acks[0] && m.acks[1] && !m.acks[2],
              $sformatf("abort read: 2 ACKed bytes + 1 dummy byte NACKed + STOP (%s)", m.convert2string()));
        end
        chk_rd(R_FIFO_STAT, 2 << 8, 32'h3F << 8, "CRX holds the 2 real bytes (CRX not flushed by an abort)");
        wr(R_CTRL, 32'h0000_0203);                            // CRX_FLUSH
        chk_rd(R_FIFO_STAT, 0, '1, "CRX_FLUSH");
      end
      AB_WAITBUS: begin                                      // not owning the bus (waiting for bus_free): 4.4.9 last rule
        bit ok;
        ext_write(7'h51, '{8'h00}, ok, 0);                   // the external controller keeps the bus (no STOP)
        push_cmds('{c_start(7'h50, 0), c_wr(8'h01), c_wr(8'h02, 1)});
        #(20us);
        chk_rd(R_STATUS, 32'h1, 32'h3, "STATUS: BUS_BUSY, controller not active while waiting for the bus");
        wr(R_CTRL, 32'h0001_0003);
        wait_cycles(4);
        chk_rd(R_CTRL, 32'h0000_0003, 32'h0003_001F, "CTRL.ABORT self-cleared");
        chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO flushed by ABORT while waiting for the bus");
        chk_cstat(7'h08, "ABORT while waiting for the bus");
        ext_write(7'h51, '{8'h01}, ok);                      // the external controller ends its transaction
        #(50us);
        chk_ev(I_CMPL, 0, "ABORT while waiting for the bus: no CMPL");
        chk_rd(R_STATUS, 0, 32'h2, "the controller never started");
      end
      AB_READ_ADDR: begin                                    // ABORT during the address byte of a read: after the
        push_cmds('{c_start(7'h50, 1), c_rd(), c_rd(0, 1)}); // address ACK one dummy byte with NACK, then STOP
        wait (side.b_bit_cnt == 3 && side.d_ctrl_active === 1'b1);
        wr(R_CTRL, 32'h0001_0003);
        wait_cmpl(7'h08, "abort in the read address byte");
        begin
          i2c_item m = p_sequencer.sb.last_ctrl_msg;
          chk(m.read && m.stop && m.data.size() == 1 && !m.acks[0],
              $sformatf("abort in the address of a read: one byte NACKed then STOP (%s)", m.convert2string()));
        end
        chk_rd(R_FIFO_STAT, 0, 32'h3F3F, "dummy byte not pushed to CRX, CMD flushed");
      end
      AB_READ_INFLIGHT: begin                                // abort forces the NACK of the current byte or a dummy read
        push_cmds('{c_start(7'h50, 1), c_rd(), c_rd(), c_rd(), c_rd()});
        wait_lvl(1, 1);
        wr(R_CTRL, 32'h0001_0003);
        wait_cmpl(7'h08, "abort in flight");
        begin
          i2c_item m = p_sequencer.sb.last_ctrl_msg;
          chk(m.stop && !m.acks[m.acks.size() - 1], $sformatf("abort in flight: last byte NACKed then STOP (%s)", m.convert2string()));
        end
        wr(R_CTRL, 32'h0000_0303);                            // CMD + CRX flush
      end
    endcase
  endtask
endclass : i2c_ctrl_abort_vseq

// ---------------------------------------------------------------- VP-CTL-13: CTRL_EN -> 0 while owning the bus (4.4.9)
class i2c_ctrl_disable_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_disable_vseq)
  function new(string name = "i2c_ctrl_disable_vseq"); super.new(name); endfunction

  task main();
    push_cmds('{c_start(7'h51, 0), c_wr(8'h10)});
    wait_ev(I_CMD_STALL, 2ms, 1, "stall before CTRL_EN = 0");
    wr(R_CTRL, 32'h1);
    wait_cmpl(7'h08, "CTRL_EN -> 0");
    chk_rd(R_STATUS, 0, 32'h2, "STATUS.CTRL_ACTIVE = 0");
    wr(R_CTRL, 32'h3);
    wait_dut_idle();
  endtask
endclass : i2c_ctrl_disable_vseq

// ---------------------------------------------------------------- VP-CTL-14: START byte (4.4.8, 6.7)
class i2c_ctrl_start_byte_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_start_byte_vseq)
  function new(string name = "i2c_ctrl_start_byte_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] q[$], d[$];
    rnd_bytes(2, d);
    wr(R_CTRL, 32'h13);                                       // the scoreboard checks the prefix of every START
    ctrl_write(7'h50, '{8'h08, d[0], d[1]});
    ctrl_write_read(7'h50, '{8'h08}, 2, q);                  // the Sr inside the transfer has no prefix
    chk(q[0] == d[0] && q[1] == d[1], "START byte transfers: data");
    ctrl_read(7'h51, 1, q);
    wr(R_CTRL, 32'h03);
  endtask
endclass : i2c_ctrl_start_byte_vseq

// ---------------------------------------------------------------- VP-CTL-15: general call / Device ID read (6.8, 6.9)
class i2c_ctrl_gcall_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_gcall_vseq)
  function new(string name = "i2c_ctrl_gcall_vseq"); super.new(name); endfunction

  task main();
    i2c_ip_ctrl_cmd_vseq cs;
    // general call "software reset" with NAKOK (6.8): nobody answers, no abort
    cs = i2c_ip_ctrl_cmd_vseq::type_id::create("gc_nakok");
    cs.cmds = '{13'h1100, 13'h1206}; cs.what = "general call with NAKOK";
    run_seq(cs);
    chk_ev(I_NACK, 0, "general call with NAKOK: no NACK event");
    // general call without NAKOK: NACK abort
    cs = i2c_ip_ctrl_cmd_vseq::type_id::create("gc");
    cs.cmds = '{c_start(7'h00, 0), c_wr(8'h06, 1)}; cs.exp = CMD_EXP_NACK; cs.exp_cstat = 7'h01; cs.what = "general call without NAKOK";
    run_seq(cs);
    // Device ID read of an external device (6.9): the VIP slaves have none -> NACK_ADDR
    cs = i2c_ip_ctrl_cmd_vseq::type_id::create("devid");
    // (the whole list must fit the CMD FIFO: commands queued after the NACK flush would be orphans)
    if (depth >= 6) cs.cmds = '{13'h1F8, 13'h0A0, 13'h1F9, 13'h400, 13'h400, 13'hE00};
    else            cs.cmds = '{13'h1F8, 13'h0A0, 13'h1F9, 13'hE00};
    cs.exp = CMD_EXP_NACK; cs.exp_cstat = 7'h01;
    cs.what = "Device ID read of a device without Device ID";
    run_seq(cs);
  endtask
endclass : i2c_ctrl_gcall_vseq

// ---------------------------------------------------------------- VP-CTL-16: Hs-mode controller (4.4.7, 6.6, 8.6)
//   expects i2c_ip_hs_cfg_vseq to have programmed the Hs timing and HS_EN
class i2c_ctrl_hs_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_hs_vseq)
  function new(string name = "i2c_ctrl_hs_vseq"); super.new(name); endfunction

  rand int unsigned n = 6;
  constraint c_def { soft n == 6; n inside {[1:16]}; }

  task main();
    i2c_ip_timing_meas_vseq meas = i2c_ip_timing_meas_vseq::type_id::create("meas");
    bit [7:0] d[$], q[$];
    int unsigned lat_hs = HS_SPK + 3;
    p_sequencer.side.clear_seen();
    meas.hs_only = 1;
    rnd_bytes(n, d);
    d.push_front(8'h70);
    fork
      run_seq(meas);
      begin
        ctrl_write(7'h50, d);
        ctrl_write_read(7'h50, '{8'h70}, n, q);
        meas.stop_req = 1;
      end
    join
    foreach (q[i]) chk(q[i] == d[i + 1], $sformatf("Hs read back byte %0d", i));
    chk_ev(I_HS_DET, 1, "HS_DET after the controller code:");
    w1c(32'h1 << I_HS_DET);
    chk(p_sequencer.side.seen_hs, "hs_mode_o = 1 during the Hs transfer");
    chk(p_sequencer.side.seen_cs_en, "scl_cs_en_o used in Hs-mode");
    chk(side.hs_mode === 1'b0, "hs_mode_o = 0 after STOP");
    chk_rd(R_STATUS, 0, 32'h40, "STATUS.HS_ACTIVE = 0 after STOP");
    `uvm_info("TIMING", $sformatf("Hs: tLOW mode %0.0f ns, tHIGH mode %0.0f ns, tHD;DAT %0.0f ns",
              meas.qmode(meas.m_low) / 1ns, meas.qmode(meas.m_high) / 1ns, meas.qmin(meas.m_hddat) / 1ns), UVM_LOW)
    chk(meas.qmode(meas.m_low) >= ((HS_L + lat_hs) * T - T) * 1ns && meas.qmode(meas.m_low) <= ((HS_L + lat_hs) * T + T) * 1ns,
        $sformatf("Hs tLOW = (HS SCL_LOW + LAT_HS) T: %0.0f ns", meas.qmode(meas.m_low) / 1ns));
    chk(meas.qmode(meas.m_high) >= ((HS_H + lat_hs) * T - T) * 1ns && meas.qmode(meas.m_high) <= ((HS_H + lat_hs) * T + T) * 1ns,
        $sformatf("Hs tHIGH = (HS SCL_HIGH + LAT_HS) T: %0.0f ns", meas.qmode(meas.m_high) / 1ns));
    chk(meas.qmin(meas.m_hddat) <= 70ns, $sformatf("Hs tHD;DAT %0.0f ns <= 70 ns (4.11.1)", meas.qmin(meas.m_hddat) / 1ns));
  endtask
endclass : i2c_ctrl_hs_vseq

// HS_EN has priority over STARTBYTE_EN (4.4.7); HS_EN = 0 while idle returns to F/S (6.6)
class i2c_ctrl_hs_prio_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_hs_prio_vseq)
  function new(string name = "i2c_ctrl_hs_prio_vseq"); super.new(name); endfunction

  task main();
    wr(R_CTRL, 32'h0000_001B);
    ctrl_write(7'h51, '{8'h00, 8'h5A});
    chk_ev(I_HS_DET, 1, "HS_DET with HS_EN + STARTBYTE_EN:");
    w1c(32'h1 << I_HS_DET);
    wr(R_CTRL, 32'h0000_0003);
    ctrl_write(7'h51, '{8'h01, 8'hA5});
    chk_ev(I_HS_DET, 0, "no HS_DET after HS_EN = 0:");
  endtask
endclass : i2c_ctrl_hs_prio_vseq

// ---------------------------------------------------------------- VP-CTL-17 (corner): Sr chain to several targets
//   S A0 W p d.. Sr A1 W p d.. Sr A0 W p Sr A0 R d.. P in one transaction, random order / lengths
class i2c_ctrl_rstart_chain_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_rstart_chain_vseq)
  function new(string name = "i2c_ctrl_rstart_chain_vseq"); super.new(name); endfunction

  rand int unsigned n_seg = 3;
  constraint c_def { n_seg inside {[2:6]}; }

  task main();
    bit [12:0] c[$];
    bit [8:0]  r[$];
    int unsigned nrd = 0;
    bit [6:0]  last_a;
    bit [7:0]  last_p;
    int unsigned last_n = 0;
    bit [7:0]  wr_d[$];
    for (int s = 0; s < n_seg; s++) begin
      bit [6:0] a = cfg.slv_addr[$urandom_range(1)];
      bit [7:0] p = rnd8();
      int unsigned n = $urandom_range(1, 4);
      c.push_back(c_start(a, 0));
      c.push_back(c_wr(p));
      wr_d.delete();
      repeat (n) begin bit [7:0] x = rnd8(); wr_d.push_back(x); c.push_back(c_wr(x)); end
      last_a = a; last_p = p; last_n = n;
    end
    // final segment: read back the last written bytes through Sr
    c.push_back(c_start(last_a, 0)); c.push_back(c_wr(last_p)); c.push_back(c_start(last_a, 1));
    nrd = last_n;
    for (int i = 0; i < nrd; i++) c.push_back(c_rd(0, i == nrd - 1));
    fork push_cmds(c); drain_crx(nrd, r); join
    wait_cmpl(0, $sformatf("Sr chain of %0d segments", n_seg + 1));
    foreach (r[i]) chk(r[i][7:0] == wr_d[i] && r[i][8] == (i == nrd - 1), $sformatf("Sr chain read-back byte %0d = %03h, written %02h", i, r[i], wr_d[i]));
    chk(p_sequencer.side.n_stop > 0, "one STOP at the end");
  endtask
endclass : i2c_ctrl_rstart_chain_vseq

// ---------------------------------------------------------------- VP-CTL-18 (corner): back-to-back transactions queued
//   several complete transactions (S..P) queued at once: one CMPL per STOP, tBUF between them (scoreboard + VIP)
class i2c_ctrl_back2back_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ctrl_back2back_vseq)
  function new(string name = "i2c_ctrl_back2back_vseq"); super.new(name); endfunction

  rand int unsigned n_txn = 3;
  constraint c_def { n_txn inside {[2:5]}; }

  task main();
    bit [12:0] c[$];
    int unsigned c0 = p_sequencer.side.ev_cnt[I_CMPL];
    int unsigned p0 = p_sequencer.side.n_stop;
    for (int t = 0; t < n_txn; t++) begin
      c.push_back(c_start(cfg.slv_addr[t % 2], 0));
      c.push_back(c_wr(8'h40 + t));
      c.push_back(c_wr(rnd8(), 1));
    end
    push_cmds(c);
    fork begin
      fork
        wait (p_sequencer.side.n_stop >= p0 + n_txn);
        begin #(10ms); chk(0, "STOP of every queued transaction"); end
      join_any
      disable fork;
    end join
    wait_cmpl(0, "back-to-back transactions");
    chk(p_sequencer.side.n_stop - p0 == n_txn, $sformatf("%0d STOPs for %0d queued transactions", p_sequencer.side.n_stop - p0, n_txn));
    chk(p_sequencer.side.ev_cnt[I_CMPL] - c0 >= 1, "CMPL raised");
    chk_rd(R_FIFO_STAT, 0, 32'h3F, "CMD FIFO consumed");
  endtask
endclass : i2c_ctrl_back2back_vseq
