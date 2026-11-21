// -----------------------------------------------------------------------------
// Random sequences
//   i2c_rand_traffic_vseq          constrained-random mix of controller and target traffic. The layer-1 transfers are
//                                  created through the factory, so a test can replace them (type override).
//   i2c_ip_ctrl_xfer_stretch_vseq  controller transfer with random clock stretching by the external target
//   i2c_rand_reg_vseq              random register accesses (random values / strobes / offsets) with mirror checks
// -----------------------------------------------------------------------------

class i2c_ip_ctrl_xfer_stretch_vseq extends i2c_ip_ctrl_xfer_vseq;
  `uvm_object_utils(i2c_ip_ctrl_xfer_stretch_vseq)
  function new(string name = "i2c_ip_ctrl_xfer_stretch_vseq"); super.new(name); endfunction

  rand int unsigned stretch_ns = 0;
  constraint c_stretch { stretch_ns dist { 0 := 1, [200:1000] := 2, [1001:6000] := 2 }; }

  task main();
    foreach (cfg.i2c_slv_cfg[i]) cfg.i2c_slv_cfg[i].stretch_ns = stretch_ns;
    super.main();
    foreach (cfg.i2c_slv_cfg[i]) cfg.i2c_slv_cfg[i].stretch_ns = 0;
  endtask
endclass : i2c_ip_ctrl_xfer_stretch_vseq

class i2c_rand_traffic_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_rand_traffic_vseq)
  function new(string name = "i2c_rand_traffic_vseq"); super.new(name); endfunction

  rand int unsigned n_iter = 30;
  // operation weights: DUT controller write / read / write-read, external write / read / write-read to the target
  rand int unsigned w_cwr = 2, w_crd = 1, w_cwrrd = 2, w_ewr = 2, w_erd = 1, w_ewrrd = 1;
  rand int unsigned max_len = 32;      // controller transfers (beyond FIFO_DEPTH allowed)
  constraint c_def {
    soft n_iter == 30; n_iter inside {[1:500]};
    soft w_cwr == 2; soft w_crd == 1; soft w_cwrrd == 2; soft w_ewr == 2; soft w_erd == 1; soft w_ewrrd == 1;
    w_cwr + w_crd + w_cwrrd + w_ewr + w_erd + w_ewrrd > 0;
    w_cwr <= 10; w_crd <= 10; w_cwrrd <= 10; w_ewr <= 10; w_erd <= 10; w_ewrrd <= 10;
    soft max_len == 32; max_len inside {[1:96]};
  }

  int unsigned n_ctrl, n_ext;

  task main();
    `uvm_info("RAND", $sformatf("%0d transactions at %s, weights C(wr %0d rd %0d wrrd %0d) E(wr %0d rd %0d wrrd %0d), max_len %0d",
              n_iter, spd.name(), w_cwr, w_crd, w_cwrrd, w_ewr, w_erd, w_ewrrd, max_len), UVM_LOW)
    for (int it = 0; it < n_iter; it++) begin
      int unsigned op;
      randcase
        w_cwr:   op = 0;
        w_crd:   op = 1;
        w_cwrrd: op = 2;
        w_ewr:   op = 3;
        w_erd:   op = 4;
        w_ewrrd: op = 5;
      endcase
      if (op < 3) ctrl_op(op);
      else        ext_op(op);
      w1c(EVENT_MASK);
    end
    chk_ev(I_ARB_LOST, 0, "no arbitration loss in the random run:");
    `uvm_info("RAND", $sformatf("%0d controller / %0d external transfers done", n_ctrl, n_ext), UVM_LOW)
  endtask

  task ctrl_op(int unsigned op);
    i2c_ip_ctrl_xfer_vseq x = i2c_ip_ctrl_xfer_vseq::type_id::create($sformatf("cx%0d", n_ctrl));
    int unsigned ml = max_len;
    if (!x.randomize() with { kind == i2c_ip_ctrl_xfer_e'(op); no_ptr == 0;
                               data.size() inside {[1:ml]}; rd_len inside {[1:ml]}; })
      `uvm_fatal("RAND", "ctrl xfer")
    run_seq(x);
    n_ctrl++;
  endtask

  task ext_op(int unsigned op);
    if (op == 3) begin                                        // write, TRX drained while the transfer runs
      i2c_ip_ext_xfer_vseq   x   = i2c_ip_ext_xfer_vseq::type_id::create($sformatf("ex%0d", n_ext));
      i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
      if (!x.randomize() with { kind == EX_WRITE; dst inside {DST_OWN0, DST_OWN1}; data.size() inside {[1:16]}; })
        `uvm_fatal("RAND", "ext xfer")
      x.exp_ack = 1;
      fork
        begin run_seq(x); col.stop_req = 1; end
        run_seq(col);
      join
      chk(col.q.size() == x.data.size() + 2, $sformatf("random external write: %0d TRX entries for %0d bytes", col.q.size(), x.data.size()));
    end else begin                                            // read / write-read served on TX_REQ, part preloaded
      i2c_ip_ext_xfer_vseq    x   = i2c_ip_ext_xfer_vseq::type_id::create($sformatf("ex%0d", n_ext));
      i2c_ip_ttx_service_vseq srv = i2c_ip_ttx_service_vseq::type_id::create("srv");
      i2c_ip_trx_collect_vseq col = i2c_ip_trx_collect_vseq::type_id::create("col");
      bit [7:0] d[$];
      if (!x.randomize() with { kind == ((op == 4) ? EX_READ : EX_WRITE_READ); dst inside {DST_OWN0, DST_OWN1}; rd_len inside {[1:16]}; })
        `uvm_fatal("RAND", "ext xfer")
      x.exp_ack = 1;
      rnd_bytes(x.rd_len, d);
      srv.src = d;
      if ($urandom_range(1)) begin
        int unsigned k = $urandom_range(0, (x.rd_len < depth) ? x.rd_len : depth);
        repeat (k) wr(R_TTXDATA, srv.src.pop_front());
      end
      fork
        run_seq(srv);
        begin run_seq(x); srv.stop_req = 1; end
      join
      chk(x.rdata == d, "random external read from the DUT target");
      col.stop_req = 1;
      run_seq(col);
    end
    n_ext++;
  endtask
endclass : i2c_rand_traffic_vseq

class i2c_rand_reg_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_rand_reg_vseq)
  function new(string name = "i2c_rand_reg_vseq"); super.new(name); endfunction

  rand int unsigned n_iter = 300;
  constraint c_def { soft n_iter == 300; n_iter inside {[1:5000]}; }

  task main();
    i2c_ip_reg rw[$];
    bit [31:0] v;
    bit e;
    rw = '{ral.INTR_EN, ral.FIFO_CTRL, ral.TADDR, ral.TCFG, ral.HSCFG, ral.TIM_SCL, ral.TIM_STA, ral.TIM_STO, ral.TIM_DAT,
           ral.HS_TIM_SCL, ral.HS_TIM_STA, ral.HS_TIM_STO, ral.HS_TIM_DAT, ral.FILTER, ral.TIMEOUT, ral.BUSIDLE,
           ral.DEVID, ral.DMA_BLK};
    wr(R_CTRL, 32'h0);
    repeat (n_iter) begin
      randcase
        6: begin                                              // random write, random strobes, to a RW register
          i2c_ip_reg r = rw[$urandom_range(rw.size() - 1)];
          bit [31:0] d = $urandom;
          bit [3:0]  s = $urandom;
          if (r == ral.FIFO_CTRL) for (int k = 0; k < 4; k++) d[8*k +: 6] = d[8*k +: 6] % depth;
          if (r == ral.DMA_BLK) d[5:0] = d[5:0] % (depth + 1);
          wr(r.get_offset() | 12'($urandom_range(3)), d, s);
        end
        4: begin                                              // front-door mirror check of a random RW register
          uvm_status_e st;
          i2c_ip_reg r = rw[$urandom_range(rw.size() - 1)];
          r.mirror(st, UVM_CHECK, .parent(this));
          chk(st == UVM_IS_OK, {"mirror of ", r.get_name()});
        end
        1: begin                                              // RO register write (ignored), read
          i2c_ip_reg ro[$] = '{ral.STATUS, ral.FIFO_STAT, ral.TSTAT, ral.PARAM, ral.VERSION};
          i2c_ip_reg r = ro[$urandom_range(ro.size() - 1)];
          wr_e(r.get_offset(), $urandom, e, $urandom);
          chk(!e, {r.get_name(), " write: no PSLVERR"});
          rd(r.get_offset(), v);
        end
        1: begin                                              // unmapped access: PSLVERR (scoreboard rule)
          bit [11:0] a = $urandom_range(1) ? 12'($urandom_range('h080, 'h0F7)) : 12'($urandom_range('h100, 'hFFF));
          if ($urandom_range(1)) wr_e(a, $urandom, e, $urandom); else rd_e(a, v, e);
          chk(e, $sformatf("unmapped 0x%03h: PSLVERR", a));
        end
      endcase
    end
    foreach (rw[i]) begin                                     // final full mirror check, then back to reset values
      uvm_status_e st;
      rw[i].mirror(st, UVM_CHECK, .parent(this));
      wr(rw[i].get_offset(), 32'(rw[i].get_reset()));
    end
  endtask
endclass : i2c_rand_reg_vseq
