// -----------------------------------------------------------------------------
// Layer 1 - utility sequences
//   i2c_ip_timing_meas_vseq  bus timing measurement running in parallel with traffic (until stop_req)
//   i2c_ip_recover_vseq      bus recovery: bus clear + idle detection (6.14)
//   i2c_ip_presetn_vseq      PRESETn pulse
//   i2c_ip_eot_vseq          end-of-test checks: bus released, no transfer active, FIFOs empty
// -----------------------------------------------------------------------------

class i2c_ip_timing_meas_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_timing_meas_vseq)

  bit      stop_req;
  bit      hs_only;                  // record only while hs_mode_o = 1
  realtime m_low[$], m_high[$], m_hddat[$], m_hdsta[$], m_susto[$], m_susta[$], m_buf[$];

  function new(string name = "i2c_ip_timing_meas_vseq"); super.new(name); endfunction

  // SCL low / high times, START hold, STOP setup, Sr setup, bus free, and the SDA change delay after the SCL fall
  // for the address bits (falls 1..8 after a START: the controller drives SDA)
  task main();
    realtime t_fall = 0, t_rise = 0, t_start = 0, t_stop = -1;
    int      nfall = 99;
    logic    scl_p = 1, sda_p = 1;
    fork begin
      fork
        wait (stop_req);
        forever begin
          logic s, d;
          bit   rec;
          @(bus.scl or bus.sda);
          s = bus.scl; d = bus.sda;
          rec = !hs_only || side.hs_mode === 1'b1;
          if (scl_p === 1'b1 && s === 1'b0) begin                      // SCL fall
            if (nfall == 0 && rec) m_hdsta.push_back($realtime - t_start);
            else if (t_rise > 0 && rec && nfall < 99) m_high.push_back($realtime - t_rise);
            t_fall = $realtime;
            if (nfall < 99) nfall++;
          end else if (scl_p === 1'b0 && s === 1'b1) begin             // SCL rise
            if (t_fall > 0 && rec && nfall < 99) m_low.push_back($realtime - t_fall);
            t_rise = $realtime;
          end else if (s === 1'b1 && sda_p === 1'b1 && d === 1'b0) begin  // START / Sr
            if (nfall < 99 && rec) m_susta.push_back($realtime - t_rise);
            else if (t_stop >= 0) m_buf.push_back($realtime - t_stop);
            t_start = $realtime; nfall = 0;
          end else if (s === 1'b1 && sda_p === 1'b0 && d === 1'b1) begin  // STOP
            if (rec) m_susto.push_back($realtime - t_rise);
            t_stop = $realtime; nfall = 99;
          end else if (s === 1'b0 && d !== sda_p && nfall >= 1 && nfall <= 8 && rec) begin
            m_hddat.push_back($realtime - t_fall);                     // controller-driven address bit
          end
          scl_p = s; sda_p = d;
        end
      join_any
      disable fork;
    end join
  endtask

  static function realtime qmin(realtime q[$]);
    realtime m = 1s;
    foreach (q[i]) if (q[i] < m) m = q[i];
    return m;
  endfunction

  // the most frequent value (exact edges, no rise/fall time)
  static function realtime qmode(realtime q[$]);
    int unsigned cnt[realtime];
    realtime best = 0;
    int unsigned bc = 0;
    foreach (q[i]) cnt[q[i]] = cnt.exists(q[i]) ? cnt[q[i]] + 1 : 1;
    foreach (cnt[k]) if (cnt[k] > bc) begin bc = cnt[k]; best = k; end
    return best;
  endfunction
endclass : i2c_ip_timing_meas_vseq

class i2c_ip_recover_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_recover_vseq)
  function new(string name = "i2c_ip_recover_vseq"); super.new(name); endfunction
  task main(); recover(); endtask
endclass : i2c_ip_recover_vseq

class i2c_ip_presetn_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_presetn_vseq)
  int unsigned cycles = 5;
  function new(string name = "i2c_ip_presetn_vseq"); super.new(name); endfunction
  task main();
    cfg.crif.pulse(cycles);
    wait_cycles(5);
    p_sequencer.sb.hw_reset();
    ral.reset();                      // register model mirror back to the reset values
    p_sequencer.ext_owns_bus = 0;
  endtask
endclass : i2c_ip_presetn_vseq

class i2c_ip_eot_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_eot_vseq)
  bit chk_fifos = 1;
  function new(string name = "i2c_ip_eot_vseq"); super.new(name); endfunction
  task main();
    bit [31:0] st, fs;
    wait_bus_free(5us, 10ms);
    chk(side.scl_oe === 1'b0 && side.sda_oe === 1'b0, "end of test: SCL / SDA released by the DUT");
    rd(R_STATUS, st);
    chk((st & 32'hAE) == 0, $sformatf("end of test: no transfer / hold / bus clear active (STATUS = 0x%08h)", st));
    if (chk_fifos) begin
      rd(R_FIFO_STAT, fs);
      chk((fs & 32'h3F00_3F3F) == 0, $sformatf("end of test: CMD / CRX / TRX empty (FIFO_STAT = 0x%08h)", fs));
    end
  endtask
endclass : i2c_ip_eot_vseq
