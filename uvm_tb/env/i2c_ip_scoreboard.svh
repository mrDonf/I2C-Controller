// -----------------------------------------------------------------------------
// i2c_ip_scoreboard - reference model of apb_i2c built from the specification (black box)
//
//   Inputs
//     apb_imp  : every completed APB transfer (CPU and DMAC) from the APB VIP monitor
//     bus_imp  : every I2C message (S/Sr ... Sr/P) decoded by the I2C VIP monitor
//     side_imp : START pulses / interrupt events of the sideband monitor (target configuration is latched at START)
//
//   Controller model (spec 4.4): the CMD stream written over APB is replayed against the bus messages driven by the
//     DUT controller (debug_ctrl_active_o): START/Sr + address, data written, READ acknowledges (ACK / NACK of the
//     command), STOP placement, NACK abort (NAKOK), command errors E2-E5 with the abort sequence (STOP, or dummy read
//     with NACK + STOP), Hs controller code / START byte prefix. Bytes read are predicted into CRX {LAST, DATA}
//     (LAST = NACK | STOP) and compared with the CRXDATA reads (any order of read vs. prediction).
//     The address ACK of a DUT controller message must come from a VIP slave only (R4: the own target never answers).
//   Target model (spec 4.5): address decode table of 4.5.1 (own addresses, GC, START byte, CBUS / reserved, Hs code,
//     10-bit, Device ID), TNACK / NOSTRETCH, TRX entries {TAG, DATA} with the START / Sr / STOP tags of 4.5.2, TTX
//     bytes on the bus (0xFF on underflow with NOSTRETCH), Device ID bytes (wrap-around).
//   APB rules (spec 3.7): unmapped offsets answer PSLVERR and read 0; a read with PSLVERR returns 0; mapped,
//     non-FIFO registers never answer PSLVERR; data registers have their reserved bits at 0.
// -----------------------------------------------------------------------------
`uvm_analysis_imp_decl(_apb)
`uvm_analysis_imp_decl(_bus)
`uvm_analysis_imp_decl(_side)

typedef enum { TK_NONE, TK_OWN0, TK_OWN1, TK_GC, TK_GC_OFF, TK_DEVW, TK_DEVR, TK_SB, TK_HS, TK_CBUS_RSVD, TK_10B,
               TK_RSVD, TK_DIS } tgt_kind_e;

class i2c_ip_msg_info extends uvm_object;
  `uvm_object_utils(i2c_ip_msg_info)
  i2c_item       m;
  bit            by_ctrl;           // message driven by the DUT controller
  bit            ctrl_prefix;       // Hs controller code / START byte
  bit            hs;                // message in Hs-mode
  tgt_kind_e     kind;              // target decode (external messages)
  bit            dut_ack;
  i2c_ip_speed_e speed;
  function new(string name = "i2c_ip_msg_info"); super.new(name); endfunction
endclass : i2c_ip_msg_info

class i2c_ip_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(i2c_ip_scoreboard)

  uvm_analysis_imp_apb  #(apb_item, i2c_ip_scoreboard)         apb_imp;
  uvm_analysis_imp_bus  #(i2c_item, i2c_ip_scoreboard)         bus_imp;
  uvm_analysis_imp_side #(i2c_ip_side_item, i2c_ip_scoreboard) side_imp;
  uvm_analysis_port #(i2c_ip_msg_info) ap_msg;

  i2c_ip_env_cfg      cfg;
  i2c_ip_side_monitor side;

  // ---- register shadows (spec 5.2)
  bit        en, ctrl_en, tgt_en, hs_en, sb_en;
  bit [2:0]  mcode = 3'b001;
  bit [6:0]  addr0, addr1;
  bit        addr0_en, addr1_en, gc_en, nostretch, tnack, devid_en;
  bit [23:0] devid;

  typedef struct { realtime t; bit en, tgt_en, gc_en, devid_en, addr0_en, addr1_en; bit [6:0] addr0, addr1; } snap_t;
  protected snap_t snaps[$];

  // ---- controller model
  bit [12:0] cmd_q[$];
  bit        c_in_txn, c_hs, c_abort_pend;
  bit [8:0]  crx_exp[$], crx_act[$];
  // ---- target model
  bit [9:0]  trx_exp[$], trx_act[$];
  bit [7:0]  ttx_q[$];
  bit        t_addressed, t_addr_seen, t_armed;
  int        t_devid_idx;

  // ---- results / expectations visible to the tests
  i2c_item     last_ctrl_msg, last_tgt_msg;
  int unsigned n_ctrl_msg, n_tgt_msg, n_crx_ok, n_trx_ok, n_ttx_ok, n_devid_ok, n_cmd_err_seen, n_ctrl_lost, n_err;
  int unsigned n_exp_taddr, n_exp_tstop, n_exp_gc_swrst, n_exp_trx_ovf, n_exp_ttx_unf, n_exp_hs_det, n_exp_ttx_done;

  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    apb_imp  = new("apb_imp", this);
    bus_imp  = new("bus_imp", this);
    side_imp = new("side_imp", this);
    ap_msg   = new("ap_msg", this);
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    devid = cfg.p.devid_rst;
  endfunction

  protected function void err(string id, string msg);
    n_err++;
    `uvm_error(id, msg)
  endfunction

  // ================================================================ housekeeping
  // drop every prediction (tests call it after deliberate protocol abuse)
  function void resync();
    cmd_q.delete(); crx_exp.delete(); crx_act.delete(); trx_exp.delete(); trx_act.delete(); ttx_q.delete();
    c_in_txn = 0; c_hs = 0; c_abort_pend = 0;
    t_addressed = 0; t_addr_seen = 0; t_armed = 0;
  endfunction

  function int unsigned trx_level();
    return (trx_exp.size() > trx_act.size()) ? trx_exp.size() - trx_act.size() : 0;
  endfunction
  function int unsigned trx_free();
    return cfg.p.fifo_depth - trx_level();
  endfunction

  protected function void cmp_crx();
    while (crx_exp.size() != 0 && crx_act.size() != 0) begin
      bit [8:0] e = crx_exp.pop_front();
      bit [8:0] a = crx_act.pop_front();
      if (a !== e) err("SB_CRX", $sformatf("CRXDATA read {LAST=%0d, 0x%02h}, expected {LAST=%0d, 0x%02h}", a[8], a[7:0], e[8], e[7:0]));
      else n_crx_ok++;
    end
  endfunction

  protected function void cmp_trx();
    while (trx_exp.size() != 0 && trx_act.size() != 0) begin
      bit [9:0] e = trx_exp.pop_front();
      bit [9:0] a = trx_act.pop_front();
      if (a !== e) err("SB_TRX", $sformatf("TRXDATA read {TAG=%02b, 0x%02h}, expected {TAG=%02b, 0x%02h}", a[9:8], a[7:0], e[9:8], e[7:0]));
      else n_trx_ok++;
    end
  endfunction

  // PRESETn: every register back to its reset value (5.1), all predictions dropped
  function void hw_reset();
    resync();
    en = 0; ctrl_en = 0; tgt_en = 0; hs_en = 0; sb_en = 0; mcode = 3'b001;
    addr0 = 0; addr1 = 0; addr0_en = 0; addr1_en = 0; gc_en = 0; nostretch = 0; tnack = 0; devid_en = 0;
    devid = cfg.p.devid_rst;
    snaps.delete();
  endfunction

  protected function void swrst();
    resync();
    en = 0; ctrl_en = 0; tgt_en = 0;
  endfunction

  // ================================================================ APB
  static function bit mapped(bit [11:0] off);
    return (off <= 12'h07C) || (off == 12'h0F8) || (off == 12'h0FC);
  endfunction

  function void write_apb(apb_item t);
    bit [11:0] off = t.addr[11:0] & 12'hFFC;
    bit [31:0] wd  = t.strobed_data();
    bit        fifo_reg = (off == R_CMD) || (off == R_TTXDATA) || (off == R_CRXDATA) || (off == R_TRXDATA);
    if (cfg.sb_apb_check) begin
      if (!mapped(off) && !t.slverr) err("SB_APB", $sformatf("access to unmapped offset 0x%03h without PSLVERR", off));
      if (mapped(off) && !fifo_reg && t.slverr) err("SB_APB", $sformatf("unexpected PSLVERR at 0x%03h: %s", off, t.convert2string()));
      if (!t.write && t.slverr && t.data !== 0) err("SB_APB", $sformatf("read with PSLVERR must return 0: %s", t.convert2string()));
      if (!t.write && off == R_CRXDATA && t.data[31:9] !== 0) err("SB_APB", $sformatf("CRXDATA[31:9] not 0: 0x%08h", t.data));
      if (!t.write && off == R_TRXDATA && t.data[31:10] !== 0) err("SB_APB", $sformatf("TRXDATA[31:10] not 0: 0x%08h", t.data));
    end
    if (t.slverr) return;                                 // errors have no effect (spec 3.7)
    if (t.write) begin
      case (off)
        R_CTRL: begin
          bit swr = t.strb[3] && t.data[31];
          bit act = (side != null) && side.ctrl_active_at($realtime);
          if (t.strb[0]) begin
            if ((ctrl_en && !wd[1] && (c_in_txn || act))) c_abort_pend = 1;      // CTRL_EN -> 0 while active
            if (en && !wd[0]) begin c_in_txn = 0; c_abort_pend = 0; t_addressed = 0; t_addr_seen = 0; t_armed = 0; end
            {sb_en, hs_en, tgt_en, ctrl_en, en} = wd[4:0];
          end
          if (t.strb[1]) begin
            if (wd[8])  cmd_q.delete();
            if (wd[9])  begin cmp_crx(); crx_exp.delete(); crx_act.delete(); end
            if (wd[10]) ttx_q.delete();
            if (wd[11]) begin cmp_trx(); trx_exp.delete(); trx_act.delete(); end
          end
          if (t.strb[2] && wd[16]) begin
            if (c_in_txn || act) c_abort_pend = 1;
            else cmd_q.delete();                          // abort while idle: flush only (spec 4.4.9)
          end
          if (swr) swrst();
        end
        R_CMD:     if (t.strb != 0) cmd_q.push_back(wd[12:0]);
        R_TTXDATA: if (t.strb != 0) ttx_q.push_back(wd[7:0]);
        R_TADDR: begin
          if (t.strb[0]) {addr0_en, addr0} = wd[7:0];
          if (t.strb[1]) {addr1_en, addr1} = wd[15:8];
        end
        R_TCFG:  if (t.strb[0]) {devid_en, tnack, nostretch, gc_en} = wd[3:0];
        R_HSCFG: if (t.strb[0]) mcode = wd[2:0];
        R_DEVID: begin
          if (t.strb[0]) devid[7:0]   = wd[7:0];
          if (t.strb[1]) devid[15:8]  = wd[15:8];
          if (t.strb[2]) devid[23:16] = wd[23:16];
        end
        default: ;
      endcase
    end else begin
      if (off == R_CRXDATA) begin crx_act.push_back(t.data[8:0]); if (cfg.sb_ctrl_check) cmp_crx(); end
      if (off == R_TRXDATA) begin trx_act.push_back(t.data[9:0]); if (cfg.sb_tgt_check)  cmp_trx(); end
    end
  endfunction

  // ================================================================ sideband: configuration latched at START (4.5.1)
  function void write_side(i2c_ip_side_item it);
    snap_t s;
    // CMD_ERR while the controller does not own the bus (E1 / E5 from idle): the CMD FIFO is flushed, no bus
    // activity (4.4.5); errors while owning the bus are evaluated on the bus message
    if (it.kind == SIDE_INTR && it.idx == I_CMD_ERR && !c_in_txn && !side.ctrl_active_at($realtime)) begin
      n_cmd_err_seen++;
      cmd_q.delete();
    end
    if (it.kind != SIDE_START) return;
    s.t = $realtime; s.en = en; s.tgt_en = tgt_en; s.gc_en = gc_en; s.devid_en = devid_en;
    s.addr0_en = addr0_en; s.addr1_en = addr1_en; s.addr0 = addr0; s.addr1 = addr1;
    snaps.push_back(s);
    if (snaps.size() > 32) void'(snaps.pop_front());
  endfunction

  protected function snap_t get_snap(realtime t0);
    snap_t s;
    while (snaps.size() != 0 && snaps[0].t < t0) void'(snaps.pop_front());
    if (snaps.size() != 0 && snaps[0].t <= t0 + 2us) return snaps.pop_front();
    // the DUT did not detect this START (disabled): current configuration
    s.t = t0; s.en = en; s.tgt_en = tgt_en; s.gc_en = gc_en; s.devid_en = devid_en;
    s.addr0_en = addr0_en; s.addr1_en = addr1_en; s.addr0 = addr0; s.addr1 = addr1;
    if (!en) s.en = 0;
    return s;
  endfunction

  // ================================================================ bus messages
  function void write_bus(i2c_item m);
    i2c_ip_msg_info info = i2c_ip_msg_info::type_id::create("info");
    bit part, done;
    realtime ts = m.start_time + 100ns;
    part = (side != null) && side.ctrl_active_at(ts);
    done = part && !side.ctrl_fell_between(ts, $realtime);
    info.m = m; info.speed = cfg.speed; info.hs = (side != null) && side.seen_hs && c_hs;
    if (part) begin
      info.by_ctrl = 1;
      n_ctrl_msg++;
      last_ctrl_msg = m;
      if (!done) begin
        bit [6:0] my_addr = (cmd_q.size() != 0) ? cmd_q[0][7:1] : 7'h7F;
        n_ctrl_lost++;
        `uvm_info("SB", $sformatf("DUT controller left the bus during {%s} (arbitration lost / abort without STOP)", m.convert2string()), UVM_MEDIUM)
        cmd_q.delete(); c_in_txn = 0; c_hs = 0; c_abort_pend = 0;
        if (m.addr[6:0] != my_addr || m.addr10) begin            // lost in the address byte: the target may answer
          info.by_ctrl = 0;
          tgt_msg(m, info);
        end
      end else if (cfg.sb_ctrl_check) ctrl_msg(m, info);
    end else begin
      n_tgt_msg++;
      last_tgt_msg = m;
      tgt_msg(m, info);
    end
    ap_msg.write(info);
  endfunction

  // ---------------------------------------------------------------- controller
  protected function bit cmd_error(bit [12:0] c, bit dir_rd, bit rd_ack_pend, bit rd_nack_nostop);
    return (c[C_START] && c[C_READ]) ||                      // E5
           (c[C_START] && rd_ack_pend) ||                    // E3
           (!c[C_START] && rd_nack_nostop) ||                // E4
           (!c[C_START] && (c[C_READ] != dir_rd));           // E2
  endfunction

  protected function void ctrl_msg(i2c_item m, i2c_ip_msg_info info);
    bit [12:0] c;
    bit [7:0]  got = {m.addr[6:0], m.read};
    bit        dir_rd, rd_ack_pend, rd_nack_nostop;
    int        n = m.data.size();

    if (c_abort_pend) begin                                  // user abort: completes the byte, NACK/dummy, STOP (4.4.9)
      if (m.stop) begin cmd_q.delete(); c_abort_pend = 0; c_in_txn = 0; c_hs = 0; end
      else c_in_txn = 1;
      return;
    end
    // ---- prefix of a START from idle (4.4.7 / 4.4.8)
    if (!c_in_txn && (hs_en || sb_en)) begin
      bit [7:0] pre = hs_en ? {5'b00001, mcode} : 8'h01;
      info.ctrl_prefix = 1;
      if (m.addr10 || got != pre)
        err("SB_CTRL", $sformatf("%s expected first after START (0x%02h), bus: %s", hs_en ? "Hs controller code" : "START byte", pre, m.convert2string()));
      if (n != 0 || m.stop)
        err("SB_CTRL", $sformatf("prefix byte must be followed by a repeated START: %s", m.convert2string()));
      c_in_txn = 1;
      c_hs     = hs_en;
      return;
    end
    if (cmd_q.size() == 0) begin
      err("SB_CTRL", $sformatf("DUT controller message without any command: %s", m.convert2string()));
      c_in_txn = !m.stop;
      return;
    end
    c = cmd_q.pop_front();
    if (!c[C_START] || c[C_READ])
      err("SB_CTRL", $sformatf("command 0x%04h (no START / START+READ) reached the bus: %s", c, m.convert2string()));
    if (m.addr10 || got != c[7:0])
      err("SB_CTRL", $sformatf("address byte 0x%02h expected, bus: %s", c[7:0], m.convert2string()));
    // R4: only an external target may acknowledge the DUT controller
    if (m.addr_ack != (!m.addr10 && cfg.is_vip_slave(m.addr[6:0])))
      err("SB_CTRL", $sformatf("address ACK=%0d unexpected (only VIP slaves %p answer): %s", m.addr_ack, cfg.slv_addr, m.convert2string()));
    dir_rd = c[0];
    if (!m.addr_ack && !c[C_NAKOK]) begin                    // NACK abort: STOP, CMD flush
      if (n != 0 || !m.stop) err("SB_CTRL", $sformatf("address NACK must end with STOP: %s", m.convert2string()));
      cmd_q.delete(); c_in_txn = 0; c_hs = 0;
      return;
    end
    if (c[C_STOP]) begin
      if (!m.stop || n > (dir_rd ? 1 : 0)) err("SB_CTRL", $sformatf("STOP expected after the address: %s", m.convert2string()));
      c_in_txn = 0; c_hs = 0;
      return;
    end
    rd_ack_pend    = dir_rd && m.addr_ack;
    rd_nack_nostop = 0;
    for (int i = 0; i < n; i++) begin
      if (cmd_q.size() == 0) begin
        err("SB_CTRL", $sformatf("byte %0d on the bus without a command: %s", i, m.convert2string()));
        break;
      end
      c = cmd_q[0];
      if (cmd_error(c, dir_rd, rd_ack_pend, rd_nack_nostop)) begin
        n_cmd_err_seen++;
        cmd_q.delete();
        if (!rd_ack_pend) err("SB_CTRL", $sformatf("command error 0x%04h must end the transfer with STOP, bus: %s", c, m.convert2string()));
        else if (i != n - 1 || m.acks[i] || !m.stop)
          err("SB_CTRL", $sformatf("command error 0x%04h while the target transmits: one dummy read with NACK + STOP expected, bus: %s", c, m.convert2string()));
        c_in_txn = 0; c_hs = 0;
        return;
      end
      if (c[C_START]) begin
        err("SB_CTRL", $sformatf("data byte on the bus where command 0x%04h requests a repeated START: %s", c, m.convert2string()));
        cmd_q.delete(); c_in_txn = !m.stop;
        return;
      end
      void'(cmd_q.pop_front());
      if (!dir_rd) begin
        if (m.data[i] !== c[7:0]) err("SB_CTRL", $sformatf("write byte %0d = 0x%02h, command 0x%02h", i, m.data[i], c[7:0]));
        if (!m.acks[i] && !c[C_NAKOK]) begin
          if (i != n - 1 || !m.stop) err("SB_CTRL", $sformatf("data NACK must end the transfer with STOP: %s", m.convert2string()));
          cmd_q.delete(); c_in_txn = 0; c_hs = 0;
          return;
        end
      end else begin
        bit nk = c[C_NACK] | c[C_STOP];
        if (m.acks[i] != !nk) err("SB_CTRL", $sformatf("read byte %0d answered %s, command 0x%04h requires %s", i, m.acks[i] ? "ACK" : "NACK", c, nk ? "NACK" : "ACK"));
        crx_exp.push_back({nk, m.data[i]});
        cmp_crx();
        rd_ack_pend    = !nk;
        rd_nack_nostop = nk && !c[C_STOP];
      end
      if (c[C_STOP]) begin
        if (i != n - 1 || !m.stop) err("SB_CTRL", $sformatf("STOP expected after byte %0d: %s", i, m.convert2string()));
        c_in_txn = 0; c_hs = 0;
        return;
      end
    end
    if (m.stop) begin
      // a STOP without a STOP command: only after a command error detected at the next fetch (E2 / E4 / E5)
      if (cmd_q.size() != 0 && cmd_error(cmd_q[0], dir_rd, rd_ack_pend, rd_nack_nostop) && !rd_ack_pend) begin
        n_cmd_err_seen++;
        cmd_q.delete();
      end else err("SB_CTRL", $sformatf("unexpected STOP (no STOP command, no command error): %s", m.convert2string()));
      c_in_txn = 0; c_hs = 0;
    end else begin
      c_in_txn = 1;
      if (cmd_q.size() == 0 || !cmd_q[0][C_START] || cmd_error(cmd_q[0], dir_rd, rd_ack_pend, rd_nack_nostop))
        err("SB_CTRL", $sformatf("repeated START without a valid START command: %s", m.convert2string()));
    end
  endfunction

  // ---------------------------------------------------------------- target
  protected function tgt_kind_e decode(bit [7:0] b, snap_t s);
    if (!s.en || !s.tgt_en)           return TK_DIS;
    if (b == 8'h00)                    return s.gc_en ? TK_GC : TK_GC_OFF;
    if (b == 8'h01)                    return TK_SB;
    if (b[7:1] inside {[7'h01:7'h03]}) return TK_CBUS_RSVD;
    if (b[7:3] == 5'b00001)            return TK_HS;
    if (b[7:3] == 5'b11110)            return TK_10B;
    if (b == 8'hF8)                    return s.devid_en ? TK_DEVW : TK_RSVD;
    if (b == 8'hF9)                    return (t_armed && s.devid_en) ? TK_DEVR : TK_RSVD;   // 4.5.6 D7, D9
    if (b[7:3] == 5'b11111)            return TK_RSVD;
    if (s.addr0_en && b[7:1] == s.addr0) return TK_OWN0;
    if (s.addr1_en && b[7:1] == s.addr1) return TK_OWN1;
    return TK_NONE;
  endfunction

  protected function void tgt_msg(i2c_item m, i2c_ip_msg_info info);
    snap_t     s = get_snap(m.start_time);
    bit [7:0]  b = m.addr10 ? {5'b11110, m.addr[9:8], m.read} : {m.addr[6:0], m.read};
    tgt_kind_e k;
    bit        dut_ack, real_addr, exp_bus_ack;
    int        n = m.data.size();
    if (!m.repeated_start) begin t_addressed = 0; t_addr_seen = 0; t_armed = 0; end
    k = decode(b, s);
    dut_ack = (k inside {TK_OWN0, TK_OWN1, TK_GC, TK_DEVW, TK_DEVR});
    if ((k inside {TK_OWN0, TK_OWN1, TK_GC}) && nostretch && trx_free() < 2) begin
      dut_ack = 0;
      n_exp_trx_ovf++;
    end
    if (b != 8'hF9) t_armed = 0;
    if (k == TK_HS && s.en) n_exp_hs_det++;
    real_addr   = !(b == 8'h01 || b[7:3] == 5'b00001);
    exp_bus_ack = dut_ack || (!m.addr10 && cfg.is_vip_slave(m.addr[6:0]));
    info.kind = k; info.dut_ack = dut_ack;
    if (cfg.sb_tgt_check && m.addr_ack != exp_bus_ack)
      err("SB_TGT", $sformatf("address byte 0x%02h (%s): bus ACK=%0d, expected %0d", b, k.name(), m.addr_ack, exp_bus_ack));
    if (dut_ack && (k inside {TK_OWN0, TK_OWN1, TK_GC})) begin
      trx_exp.push_back({t_addr_seen ? 2'b11 : 2'b01, b});
      t_addressed = 1;
      n_exp_taddr++;
    end
    t_addr_seen |= real_addr;
    if (cfg.sb_tgt_check && dut_ack && m.addr_ack) begin
      case (k)
        TK_OWN0, TK_OWN1, TK_GC: begin
          if (!b[0]) begin                                  // target receiver
            for (int i = 0; i < n; i++) begin
              bit exp_ack = !tnack && (!nostretch || trx_free() >= 2);
              if (!exp_ack && !tnack) n_exp_trx_ovf++;
              if (m.acks[i] != exp_ack)
                err("SB_TGT", $sformatf("data byte %0d (0x%02h) to the target answered %s, expected %s (TNACK=%0d NOSTRETCH=%0d TRX free=%0d)",
                    i, m.data[i], m.acks[i] ? "ACK" : "NACK", exp_ack ? "ACK" : "NACK", tnack, nostretch, trx_free()));
              if (!exp_ack) break;
              trx_exp.push_back({2'b00, m.data[i]});
              if (k == TK_GC && i == 0 && m.data[i] == 8'h06) n_exp_gc_swrst++;
            end
          end else begin                                    // target transmitter
            for (int i = 0; i < n; i++) begin
              bit [7:0] e;
              if (ttx_q.size() != 0) e = ttx_q.pop_front();
              else if (nostretch) begin e = 8'hFF; n_exp_ttx_unf++; end
              else begin
                err("SB_TGT", $sformatf("target transmitted byte %0d (0x%02h) although TTX was never loaded", i, m.data[i]));
                e = m.data[i];
              end
              if (m.data[i] !== e) err("SB_TGT", $sformatf("target transmitted 0x%02h, TTX byte 0x%02h expected (byte %0d)", m.data[i], e, i));
              else n_ttx_ok++;
            end
            if (n != 0 && !m.acks[n-1]) n_exp_ttx_done++;
          end
        end
        TK_DEVW: begin                                      // Device ID: second byte = target address (4.5.6)
          for (int i = 0; i < n; i++) begin
            bit [6:0] a = m.data[0][7:1];
            bit rsvd    = (a[6:3] == 4'b0000) || (a[6:3] == 4'b1111);       // 4.5.6 D3: reserved never matches
            bit exp_ack = (i == 0) && !rsvd && ((s.addr0_en && a == s.addr0) || (s.addr1_en && a == s.addr1));
            if (m.acks[i] != exp_ack)
              err("SB_TGT", $sformatf("Device ID address byte %0d (0x%02h) answered %s, expected %s", i, m.data[i], m.acks[i] ? "ACK" : "NACK", exp_ack ? "ACK" : "NACK"));
            if (i == 0 && exp_ack) begin t_armed = 1; t_devid_idx = 0; end
            if (!m.acks[i]) break;
          end
        end
        TK_DEVR: begin
          t_devid_idx = 0;                                  // 4.5.6 D8: every 0xF9 starts at DEVID[23:16]
          for (int i = 0; i < n; i++) begin
            bit [7:0] e = (t_devid_idx == 0) ? devid[23:16] : (t_devid_idx == 1) ? devid[15:8] : devid[7:0];
            if (m.data[i] !== e) err("SB_TGT", $sformatf("Device ID byte %0d = 0x%02h, expected 0x%02h (DEVID 0x%06h)", i, m.data[i], e, devid));
            else n_devid_ok++;
            t_devid_idx = (t_devid_idx + 1) % 3;
          end
        end
        default: ;
      endcase
    end
    if (m.stop) begin
      if (t_addressed) begin
        if (trx_free() >= 1) trx_exp.push_back({2'b10, 8'h00});
        else n_exp_trx_ovf++;
        n_exp_tstop++;
      end
      t_addressed = 0; t_addr_seen = 0; t_armed = 0;
    end
    if (cfg.sb_tgt_check) cmp_trx();
  endfunction

  // ================================================================ end of test
  function void check_phase(uvm_phase phase);
    if (cfg.sb_ctrl_check) begin
      cmp_crx();
      if (crx_exp.size() != 0) err("SB_CRX", $sformatf("%0d byte(s) predicted for CRX were never read", crx_exp.size()));
      if (crx_act.size() != 0) err("SB_CRX", $sformatf("%0d CRXDATA read(s) without a predicted byte", crx_act.size()));
    end
    if (cfg.sb_tgt_check) begin
      cmp_trx();
      if (trx_exp.size() != 0) err("SB_TRX", $sformatf("%0d entr(ies) predicted for TRX were never read", trx_exp.size()));
      if (trx_act.size() != 0) err("SB_TRX", $sformatf("%0d TRXDATA read(s) without a predicted entry", trx_act.size()));
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `uvm_info("SB", $sformatf("ctrl msgs=%0d (left bus %0d, cmd errors %0d) tgt msgs=%0d | CRX ok=%0d TRX ok=%0d TTX ok=%0d DEVID ok=%0d | errors=%0d",
              n_ctrl_msg, n_ctrl_lost, n_cmd_err_seen, n_tgt_msg, n_crx_ok, n_trx_ok, n_ttx_ok, n_devid_ok, n_err), UVM_LOW)
  endfunction
endclass : i2c_ip_scoreboard
