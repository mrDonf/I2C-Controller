// -----------------------------------------------------------------------------
// i2c_ip_base_vseq - layer 0 of the sequence library: base of every virtual sequence.
//   Driver-model (software) API that follows the operation sequences of spec section 6. It holds no scenario: the
//   reusable layer-1 sequences (cfg / xfer / bus / util) and the layer-2 feature sequences are built on it, and the
//   tests compose them.
//
//   State shared between the sequences of a test lives in the env (cfg.speed, cfg.tgt_addr, vsqr.ext_owns_bus), so
//   every sequence instance sees the configuration programmed by the previous one.
//
//   APB      : wr / rd (raw, PSLVERR checked), wr_e / rd_e (return PSLVERR), reg_wr / reg_rd (register model)
//   checks   : chk_rd (masked compare), chk (condition), chk_ev / chk_cstat
//   init     : init_ip (6.2), set_timing, enable
//   events   : w1c, wait_ev (sideband INTR_STAT), wait_cnt (event counters), wait_irq
//   ctrl     : push_cmds (refills while the CMD FIFO is full), ctrl_write / ctrl_read / ctrl_write_read (6.3-6.5),
//              drain_crx, wait_cmpl
//   target   : drain_trx, load_ttx, VIP master accessors (ext)
//   bus      : wait_bus_free, wait_dut_idle, bb (bit-bang op), quiet / loud (checks off / on around protocol abuse),
//              recover (bus clear), count_rises
//   compose  : run_seq (start a child sequence on the virtual sequencer)
// -----------------------------------------------------------------------------
class i2c_ip_base_vseq extends uvm_sequence;
  `uvm_object_utils(i2c_ip_base_vseq)
  `uvm_declare_p_sequencer(i2c_ip_vsqr)

  i2c_ip_reg_block ral;
  i2c_ip_env_cfg   cfg;
  side_vif_t       side;
  i2c_vif_t        bus;
  int unsigned     depth;
  real             T = 10.0;                 // PCLK period (ns)
  i2c_ip_timing_t  tm;                       // F/S timing currently programmed
  i2c_ip_speed_e   spd = SPD_FM;
  bit [6:0]        own0, own1;               // DUT target addresses (cfg.tgt_addr)
  int unsigned     n_fail;

  function new(string name = "i2c_ip_base_vseq");
    super.new(name);
    set_response_queue_depth(-1);
  endfunction

  virtual task body();
    setup();
    main();
  endtask

  virtual task main(); endtask

  function void setup();
    ral   = p_sequencer.ral;
    cfg   = p_sequencer.cfg;
    side  = cfg.side_vif;
    bus   = cfg.i2c_mst_cfg.vif;
    depth = cfg.p.fifo_depth;
    T     = cfg.p.pclk_ns;
    spd   = cfg.speed;
    tm    = i2c_ip_timing(spd);
    own0  = cfg.tgt_addr[0];
    own1  = cfg.tgt_addr[1];
  endfunction

  // start a child sequence (layer-1 / layer-2) on the same virtual sequencer
  task run_seq(uvm_sequence_base s);
    s.start(p_sequencer, this);
  endtask

  // ================================================================ checks
  function void chk(bit cond, string msg);
    if (!cond) begin
      n_fail++;
      `uvm_error("VSEQ_CHK", msg)
    end else `uvm_info("VSEQ_CHK", {"ok: ", msg}, UVM_HIGH)
  endfunction

  // ================================================================ APB (CPU)
  task apb_xfer(bit write, bit [11:0] a, ref bit [31:0] d, input bit [3:0] strb, output bit err, input bit [2:0] prot = 0);
    apb_item it = apb_item::type_id::create("cpu");
    uvm_sequence_item r;
    apb_item rsp;
    start_item(it, -1, p_sequencer.apb_sqr);
    it.write = write; it.addr = a; it.data = write ? d : $urandom; it.strb = write ? strb : 4'h0; it.prot = prot; it.idle = 0;
    finish_item(it);
    get_response(r, it.get_transaction_id());
    $cast(rsp, r);
    if (!write) d = rsp.data;
    err = rsp.slverr;
  endtask

  task wr_e(bit [11:0] a, bit [31:0] d, output bit err, input bit [3:0] strb = 4'hF, input bit [2:0] prot = 0);
    apb_xfer(1, a, d, strb, err, prot);
  endtask

  task rd_e(bit [11:0] a, output bit [31:0] d, output bit err, input bit [2:0] prot = 0);
    bit [31:0] v;
    apb_xfer(0, a, v, 4'h0, err, prot);
    d = v;
  endtask

  task wr(bit [11:0] a, bit [31:0] d, bit [3:0] strb = 4'hF);
    bit e;
    wr_e(a, d, e, strb);
    if (e) begin n_fail++; `uvm_error("VSEQ_APB", $sformatf("unexpected PSLVERR writing 0x%08h to 0x%03h", d, a)) end
  endtask

  task rd(bit [11:0] a, output bit [31:0] d);
    bit e;
    rd_e(a, d, e);
    if (e) begin n_fail++; `uvm_error("VSEQ_APB", $sformatf("unexpected PSLVERR reading 0x%03h", a)) end
  endtask

  task chk_rd(bit [11:0] a, bit [31:0] exp, bit [31:0] mask = '1, string what = "");
    bit [31:0] v;
    rd(a, v);
    chk((v & mask) === (exp & mask), $sformatf("%s RD 0x%03h = 0x%08h, expected 0x%08h (mask 0x%08h)", what, a, v, exp, mask));
  endtask

  task rd_field(bit [11:0] a, int lsb, int width, output int unsigned v);
    bit [31:0] d;
    rd(a, d);
    v = (d >> lsb) & ((1 << width) - 1);
  endtask

  // poll FIFO_STAT until the level of FIFO f (0 CMD, 1 CRX, 2 TTX, 3 TRX) is >= n
  task wait_lvl(int f, int unsigned n, int unsigned gap = 10);
    int unsigned l;
    forever begin
      rd_field(R_FIFO_STAT, 8 * f, 6, l);
      if (l >= n) break;
      repeat (gap) @(posedge side.clk);
    end
  endtask

  // register model front-door (named registers)
  task reg_wr(uvm_reg r, uvm_reg_data_t v);
    uvm_status_e st;
    r.write(st, v, .parent(this));
    if (st != UVM_IS_OK) begin n_fail++; `uvm_error("VSEQ_RAL", $sformatf("write %s failed", r.get_name())) end
  endtask

  task reg_rd(uvm_reg r, output uvm_reg_data_t v);
    uvm_status_e st;
    r.read(st, v, .parent(this));
    if (st != UVM_IS_OK) begin n_fail++; `uvm_error("VSEQ_RAL", $sformatf("read %s failed", r.get_name())) end
  endtask

  // ================================================================ init (spec 6.2)
  task set_timing(i2c_ip_speed_e s);
    spd = s;
    tm  = i2c_ip_timing(s);
    reg_wr(ral.TIM_SCL, {tm.scl_high[15:0], tm.scl_low[15:0]});
    reg_wr(ral.TIM_STA, {tm.tsu_sta[15:0], tm.thd_sta[15:0]});
    reg_wr(ral.TIM_STO, {tm.tbuf[15:0], tm.tsu_sto[15:0]});
    reg_wr(ral.TIM_DAT, {tm.tsu_dat[15:0], tm.thd_dat[15:0]});
    reg_wr(ral.FILTER,  {16'h0, 8'(HS_SPK), 8'(tm.spklen)});
    cfg.set_vip_speed(s);
  endtask

  task enable(bit ctrl, bit tgt, bit hs = 0, bit sb = 0);
    wr(R_CTRL, {27'h0, sb, hs, tgt, ctrl, 1'b1});
  endtask

  // full initialisation: disabled -> timing -> target -> thresholds -> events -> enable -> bus idle
  task init_ip(i2c_ip_speed_e s = SPD_FM, bit ctrl = 1, bit tgt = 1, bit [31:0] tcfg = 0, bit [31:0] intr_en = 0,
               bit wait_idle = 1, int unsigned idle_val = 1000);
    wr(R_CTRL, 0);
    set_timing(s);
    reg_wr(ral.BUSIDLE, idle_val);
    cfg.tgt_addr[0] = own0; cfg.tgt_addr[1] = own1;
    reg_wr(ral.TADDR, {16'h0, 1'b1, own1, 1'b1, own0});
    reg_wr(ral.TCFG, tcfg);
    reg_wr(ral.FIFO_CTRL, 0);
    w1c(EVENT_MASK);
    wr(R_CSTAT, 32'h7F);
    reg_wr(ral.INTR_EN, intr_en);
    if (ctrl || tgt) enable(ctrl, tgt);
    if (wait_idle && (ctrl || tgt)) wait_dut_idle();
  endtask

  // BUS_BUSY = 0 (bus-idle detection after enable, spec 4.3.3)
  task wait_dut_idle(realtime tmo = 2ms);
    realtime t0 = $realtime;
    bit [31:0] v;
    forever begin
      rd(R_STATUS, v);
      if (!v[0]) break;
      if ($realtime - t0 > tmo) begin n_fail++; `uvm_error("VSEQ_TMO", "STATUS.BUS_BUSY stays 1") break; end
      #(5us);
    end
  endtask

  // SCL = SDA = 1 continuously for `t` (pad level)
  task wait_bus_free(realtime t = 20us, realtime tmo = 5ms);
    realtime t0 = $realtime, ts = $realtime;
    forever begin
      #(200ns);
      if (bus.scl !== 1'b1 || bus.sda !== 1'b1) ts = $realtime;
      else if ($realtime - ts >= t) break;
      if ($realtime - t0 > tmo) begin n_fail++; `uvm_error("VSEQ_TMO", "bus never becomes free") break; end
    end
  endtask

  // ================================================================ interrupts / events
  task w1c(bit [31:0] mask);
    wr(R_INTR_STAT, mask);
  endtask

  // wait until INTR_STAT[b] is set (sideband view of INTR_STAT), optional W1C
  task wait_ev(int b, realtime tmo = 10ms, bit clr = 1, string what = "");
    bit to = 0;
    fork begin
      fork
        wait (side.d_intr_stat[b] === 1'b1);
        begin #(tmo); to = 1; end
      join_any
      disable fork;
    end join
    if (to) begin n_fail++; `uvm_error("VSEQ_TMO", $sformatf("INTR_STAT[%0d] not set within %0t %s", b, tmo, what)) end
    else if (clr) w1c(32'h1 << b);
  endtask

  // wait until the event counter of INTR_STAT[b] passes `mark` (event happened again since mark)
  task wait_cnt(int b, int unsigned mark, realtime tmo = 10ms, string what = "");
    bit to = 0;
    fork begin
      fork
        wait (p_sequencer.side.ev_cnt[b] > mark);
        begin #(tmo); to = 1; end
      join_any
      disable fork;
    end join
    if (to) begin n_fail++; `uvm_error("VSEQ_TMO", $sformatf("event %0d did not occur within %0t %s", b, tmo, what)) end
  endtask

  task chk_ev(int b, bit exp, string what = "");
    bit [31:0] v;
    rd(R_INTR_STAT, v);
    chk(v[b] == exp, $sformatf("%s INTR_STAT[%0d] = %0d, expected %0d (INTR_STAT = 0x%08h)", what, b, v[b], exp, v));
  endtask

  task chk_cstat(bit [6:0] exp, string what = "", bit clr = 1);
    chk_rd(R_CSTAT, exp, 32'hFFFF_FFFF, {what, " CSTAT"});
    if (clr) wr(R_CSTAT, 32'h7F);
  endtask

  // ================================================================ controller (spec 6.3 - 6.5)
  static function bit [12:0] c_start(bit [6:0] a, bit rd_, bit stop = 0, bit nakok = 0);
    return {nakok, 1'b0, 1'b0, stop, 1'b1, a, rd_};
  endfunction
  static function bit [12:0] c_wr(bit [7:0] d, bit stop = 0, bit nakok = 0);
    return {nakok, 1'b0, 1'b0, stop, 1'b0, d};
  endfunction
  static function bit [12:0] c_rd(bit nack = 0, bit stop = 0);
    return {1'b0, nack | stop, 1'b1, stop, 1'b0, 8'h00};
  endfunction

  // write the commands; whenever the CMD FIFO is full, wait for room (polls FIFO_STAT)
  task push_cmds(bit [12:0] cmds[$], realtime poll = 5us);
    int unsigned room = 0;
    foreach (cmds[i]) begin
      while (room == 0) begin
        int unsigned lvl;
        rd_field(R_FIFO_STAT, 0, 6, lvl);
        room = depth - lvl;
        if (room == 0) #(poll);
      end
      wr(R_CMD, cmds[i]);
      room--;
    end
  endtask

  // read n bytes from CRX (polls FIFO_STAT while the transfer runs)
  task drain_crx(int unsigned n, output bit [8:0] q[$], input realtime tmo = 20ms);
    realtime t0 = $realtime;
    q.delete();
    while (q.size() < n) begin
      int unsigned lvl;
      rd_field(R_FIFO_STAT, 8, 6, lvl);
      if (lvl == 0) begin
        if ($realtime - t0 > tmo) begin n_fail++; `uvm_error("VSEQ_TMO", $sformatf("CRX: %0d of %0d bytes", q.size(), n)) return; end
        #(5us);
        continue;
      end
      repeat (lvl) begin
        bit [31:0] v;
        rd(R_CRXDATA, v);
        q.push_back(v[8:0]);
      end
    end
  endtask

  task wait_cmpl(bit [6:0] exp_cstat = 0, string what = "", realtime tmo = 20ms);
    wait_ev(I_CMPL, tmo, 1, what);
    chk_cstat(exp_cstat, what);
  endtask

  task ctrl_write(bit [6:0] a, bit [7:0] d[$], bit stop = 1, bit wait_done = 1, bit nakok = 0);
    bit [12:0] c[$];
    c.push_back(c_start(a, 0, (d.size() == 0) && stop, nakok));
    foreach (d[i]) c.push_back(c_wr(d[i], stop && (i == d.size() - 1), nakok));
    push_cmds(c);
    if (wait_done && stop) wait_cmpl(0, $sformatf("write 0x%02h", a));
  endtask

  task ctrl_read(bit [6:0] a, int unsigned n, output bit [7:0] q[$], input bit wait_done = 1);
    bit [12:0] c[$];
    bit [8:0]  r[$];
    c.push_back(c_start(a, 1));
    for (int i = 0; i < n; i++) c.push_back(c_rd(0, i == n - 1));
    fork
      push_cmds(c);
      drain_crx(n, r);
    join
    q.delete();
    foreach (r[i]) begin
      q.push_back(r[i][7:0]);
      chk(r[i][8] == (i == n - 1), $sformatf("CRXDATA.LAST of byte %0d of %0d = %0d", i, n, r[i][8]));
    end
    if (wait_done) wait_cmpl(0, $sformatf("read 0x%02h", a));
  endtask

  // register read (6.5): S addr W ptr Sr addr R D0..Dn-1 P
  task ctrl_write_read(bit [6:0] a, bit [7:0] w[$], int unsigned n, output bit [7:0] q[$]);
    bit [12:0] c[$];
    bit [8:0]  r[$];
    c.push_back(c_start(a, 0));
    foreach (w[i]) c.push_back(c_wr(w[i]));
    c.push_back(c_start(a, 1));
    for (int i = 0; i < n; i++) c.push_back(c_rd(0, i == n - 1));
    fork
      push_cmds(c);
      drain_crx(n, r);
    join
    q.delete();
    foreach (r[i]) q.push_back(r[i][7:0]);
    wait_cmpl(0, $sformatf("write-read 0x%02h", a));
  endtask

  // VIP slave memory (reference for controller data)
  function bit [7:0] slv_mem(int idx, bit [7:0] a);
    return p_sequencer.i2c_slv[idx].cfg.mem.read_byte(a);
  endfunction

  // ================================================================ target
  task drain_trx(output bit [9:0] q[$]);
    int unsigned lvl;
    q.delete();
    rd_field(R_FIFO_STAT, 24, 6, lvl);
    while (lvl != 0) begin
      repeat (lvl) begin
        bit [31:0] v;
        rd(R_TRXDATA, v);
        q.push_back(v[9:0]);
      end
      rd_field(R_FIFO_STAT, 24, 6, lvl);
    end
  endtask

  function void chk_trx(bit [9:0] got[$], bit [9:0] exp[$], string what);
    string g = "", e = "";
    foreach (got[i]) g = {g, $sformatf(" %03h", got[i])};
    foreach (exp[i]) e = {e, $sformatf(" %03h", exp[i])};
    chk(got == exp, $sformatf("%s TRX entries {%s } expected {%s }", what, g, e));
  endfunction

  task load_ttx(bit [7:0] d[$]);
    foreach (d[i]) wr(R_TTXDATA, d[i]);
  endtask

  // ================================================================ external controller (VIP master)
  // The VIP master only keeps tBUF after its own STOP: wait for a free bus (>= tBUF of the mode) before a START.
  // Not needed before a message that follows with a repeated START (bus still owned).
  task ext_bus_free();
    if (!p_sequencer.ext_owns_bus) wait_bus_free(1.3 * i2c_spec(cfg.i2c_mst_cfg.speed).t_buf * 1ns);
  endtask
  // The VIP master returns at the pad edge of its STOP; the DUT sees the STOP LAT = SPKLEN + 3 cycles later
  // (input synchronizer + glitch filter, spec 4.2). Status / FIFO checks wait this latency (+ margin) first.
  task ext_settle();
    #((tm.spklen + 3 + 10) * T * 1ns);
  endtask
  task ext_write(bit [6:0] a, bit [7:0] d[$], output bit ok, input bit stop = 1);
    ext_bus_free();
    p_sequencer.ext.write(a, d, ok, stop);
    // a message without STOP keeps the bus (SCL held low by the VIP master), also after a NACKed data byte
    p_sequencer.ext_owns_bus = !stop && (bus.scl === 1'b0);
    if (!p_sequencer.ext_owns_bus) ext_settle();
  endtask
  task ext_read(bit [6:0] a, int unsigned n, output bit [7:0] q[$], output bit ok);
    ext_bus_free();
    p_sequencer.ext.read(a, n, q, ok);
    p_sequencer.ext_owns_bus = 0;
    ext_settle();
  endtask
  // read that keeps the bus (NACK on the last byte, no STOP): the next message starts with a repeated START
  // (combined format of UM10204 3.1.10). Sent as an i2c_item on the VIP master sequencer (the VIP API read always
  // ends with STOP).
  task ext_read_nostop(bit [6:0] a, int unsigned n, output bit [7:0] q[$], output bit ok);
    i2c_item it = i2c_item::type_id::create("rd_nostop");
    ext_bus_free();
    start_item(it, -1, p_sequencer.i2c_mst_sqr);
    if (!it.randomize() with { read == 1; addr == {3'b000, a}; addr10 == 0; nread == n; stop == 0; nack_last == 1; })
      `uvm_fatal("VSEQ", "i2c_item randomize")
    finish_item(it);
    q = it.data;
    ok = it.addr_ack && !it.arb_lost;
    p_sequencer.ext_owns_bus = it.addr_ack;                  // an address NACK ends with STOP
  endtask
  task ext_write_read(bit [6:0] a, bit [7:0] ptr, int unsigned n, output bit [7:0] q[$], output bit ok);
    ext_bus_free();
    p_sequencer.ext.write_read(a, ptr, n, q, ok);
    p_sequencer.ext_owns_bus = 0;
    ext_settle();
  endtask

  // TRX entry encodings (spec 4.5.2)
  static function bit [9:0] e_start(bit [6:0] a, bit r);  return {2'b01, a, r}; endfunction
  static function bit [9:0] e_rstart(bit [6:0] a, bit r); return {2'b11, a, r}; endfunction
  static function bit [9:0] e_data(bit [7:0] d);          return {2'b00, d}; endfunction
  static function bit [9:0] e_stop();                     return {2'b10, 8'h00}; endfunction

  // ================================================================ bus-level helpers (bit-bang agent)
  task bb(bb_op_e o, bit [7:0] data = 0, real dur_ns = 0);
    p_sequencer.bbs.do_op(o, data, 0, dur_ns);
  endtask

  // VIP monitor checks off, model checks off (bus content is deliberately broken)
  function void quiet();
    cfg.i2c_mst_cfg.checks_en = 0;
    foreach (cfg.i2c_slv_cfg[i]) cfg.i2c_slv_cfg[i].checks_en = 0;
    cfg.sb_ctrl_check = 0; cfg.sb_tgt_check = 0;
  endfunction

  function void loud();
    cfg.i2c_mst_cfg.checks_en = 1;
    foreach (cfg.i2c_slv_cfg[i]) cfg.i2c_slv_cfg[i].checks_en = 1;
    p_sequencer.sb.resync();
    cfg.sb_ctrl_check = 1; cfg.sb_tgt_check = 1;
  endfunction

  // recover a hung bus with a bus clear (6.14), then wait until the DUT sees an idle bus.
  //   Step 5 of 6.14: BUSMON.SCL_F / SDA_F tell whether the bus is free. The bus clear always sends 9 clocks + STOP
  //   (4.4.10): a target that reaches its ACK slot on the STOP clock still holds SDA low, so the driver repeats the
  //   bus clear (up to n_try times) before giving up.
  int unsigned n_busclr_retry;
  task recover(int unsigned n_try = 3);
    bit [31:0] c, m;
    rd(R_CTRL, c);
    c = (c & 32'h1F) | 32'h3;                               // keep TGT_EN / HS_EN / STARTBYTE_EN, controller on
    wr(R_CTRL, c);
    for (int k = 0; k < n_try; k++) begin
      wr(R_CTRL, c | 32'h0002_0000);
      wait_ev(I_BUSCLR_DONE, 2ms, 1, "recovery bus clear");
      rd(R_BUSMON, m);
      if (m[1:0] == 2'b11) break;
      n_busclr_retry++;
      `uvm_info("RECOVER", $sformatf("BUSMON = 0x%03h after bus clear %0d: SDA / SCL still low, bus clear repeated (6.14 step 5)", m[10:0], k + 1), UVM_LOW)
    end
    chk(m[1:0] == 2'b11, $sformatf("bus released by the bus clear (BUSMON.SCL_F / SDA_F = %b)", m[1:0]));
    wait_bus_free(20us);
    wait_dut_idle();
  endtask

  int unsigned n_rise;
  task count_rises(realtime dur);
    n_rise = 0;
    fork begin
      fork
        forever begin @(posedge bus.scl); n_rise++; end
        #(dur);
      join_any
      disable fork;
    end join
  endtask

  // ================================================================ DMA (4.9, 6.15)
  function i2c_dma_op mk_op(dma_sel_e sel, bit pfc, int unsigned count, int unsigned blk, bit deny_single = 0);
    i2c_dma_op o = i2c_dma_op::type_id::create("op");
    o.sel = sel; o.pfc = pfc; o.count = count; o.blk = blk; o.deny_single = deny_single;
    return o;
  endfunction

  task wait_op(i2c_dma_op o, realtime tmo = 10ms, string what = "");
    bit to = 0;
    if (!o.done_ev.is_on()) begin                            // triggered after the final handshake (and the end handshake)
      fork begin
        fork
          o.done_ev.wait_trigger();
          begin #(tmo); to = 1; end
        join_any
        disable fork;
      end join
    end
    if (to) begin n_fail++; `uvm_error("VSEQ_TMO", $sformatf("DMA operation %s not finished: %s", what, o.log)) end
    `uvm_info("DMA", $sformatf("%s: %s", what, o.log), UVM_LOW)
  endtask

  // completion: DMA_DONE event, DMA_STAT.DONE, DMA_CTRL.EN cleared by HW (4.9.3)
  task chk_dma_done(string what);
    wait_ev(I_DMA_DONE, 1ms, 1, what);
    chk_rd(R_DMA_STAT, 32'h0000_0100, 32'h0000_0101, {what, ": DMA_STAT.DONE, no pending request"});
    chk_rd(R_DMA_CTRL, 0, 32'h1, {what, ": DMA_CTRL.EN cleared by HW"});
    wr(R_DMA_STAT, 32'h0001_0100);
  endtask

  function void chk_dma_log(i2c_dma_op o, string exp, string what);
    chk(o.log == exp, $sformatf("%s request sequence \"%s\", spec 4.9.4 \"%s\"", what, o.log, exp));
  endfunction

  // ================================================================ misc
  task wait_cycles(int unsigned n);
    repeat (n) @(posedge side.clk);
  endtask

  function bit [7:0] rnd8(); return 8'($urandom); endfunction

  function void rnd_bytes(int unsigned n, ref bit [7:0] q[$]);
    q.delete();
    repeat (n) q.push_back(8'($urandom));
  endfunction
endclass : i2c_ip_base_vseq
