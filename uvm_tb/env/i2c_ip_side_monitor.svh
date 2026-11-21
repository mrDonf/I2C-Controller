// -----------------------------------------------------------------------------
// i2c_ip_side_monitor - passive monitor of the sideband / debug ports (spec 3.3, 3.5)
//   ap   : i2c_ip_side_item for every rising edge of an INTR_STAT bit (debug_intr_stat_o), START / STOP pulses
//   ev_cnt[i]         number of 0->1 transitions of INTR_STAT[i] (tests use it as "event happened since mark")
//   ctrl_active_at(t) / ctrl_fell_between(t0, t1) : history of debug_ctrl_active_o (= STATUS.CTRL_ACTIVE), used by
//                       the scoreboard to know which bus messages were driven by the DUT controller
//   seen_* flags      : sticky observations (target stretch, controller hold, Hs-mode, current source)
// -----------------------------------------------------------------------------
typedef enum { SIDE_INTR, SIDE_START, SIDE_STOP } side_kind_e;

class i2c_ip_side_item extends uvm_sequence_item;
  `uvm_object_utils(i2c_ip_side_item)
  side_kind_e kind;
  int         idx;
  time        t;
  function new(string name = "i2c_ip_side_item"); super.new(name); endfunction
endclass : i2c_ip_side_item

class i2c_ip_side_monitor extends uvm_monitor;
  `uvm_component_utils(i2c_ip_side_monitor)

  side_vif_t vif;
  uvm_analysis_port #(i2c_ip_side_item) ap;

  int unsigned ev_cnt[22];
  int unsigned n_start, n_stop;
  bit          seen_tgt_hold, seen_ctrl_hold, seen_hs, seen_cs_en, seen_irq_err;

  protected realtime h_t[$];
  protected bit      h_v[$];

  function new(string name, uvm_component parent); super.new(name, parent); endfunction
  function void build_phase(uvm_phase phase); super.build_phase(phase); ap = new("ap", this); endfunction

  function void clear_seen();
    seen_tgt_hold = 0; seen_ctrl_hold = 0; seen_hs = 0; seen_cs_en = 0; seen_irq_err = 0;
  endfunction

  function bit ctrl_active_at(realtime t);
    bit v = 0;
    foreach (h_t[i]) begin
      if (h_t[i] > t) break;
      v = h_v[i];
    end
    return v;
  endfunction

  function bit ctrl_fell_between(realtime t0, realtime t1);
    foreach (h_t[i]) if (h_t[i] > t0 && h_t[i] <= t1 && h_v[i] == 0) return 1;
    return 0;
  endfunction

  task run_phase(uvm_phase phase);
    logic [21:0] st_p = '0;
    logic act_p = 0;
    h_t.push_back(0); h_v.push_back(0);
    forever begin
      @(vif.mon_cb);
      if (vif.rst_n !== 1'b1) begin
        st_p = '0;
        if (act_p) begin h_t.push_back($realtime); h_v.push_back(0); act_p = 0; end
        continue;
      end
      for (int i = 0; i < 22; i++) begin
        if (vif.mon_cb.d_intr_stat[i] === 1'b1 && st_p[i] !== 1'b1) begin
          i2c_ip_side_item it = i2c_ip_side_item::type_id::create("ev");
          it.kind = SIDE_INTR; it.idx = i; it.t = $time;
          ev_cnt[i]++;
          ap.write(it);
        end
      end
      st_p = vif.mon_cb.d_intr_stat;
      if (vif.mon_cb.d_start === 1'b1) begin
        i2c_ip_side_item it = i2c_ip_side_item::type_id::create("start");
        it.kind = SIDE_START; it.t = $time; n_start++;
        ap.write(it);
      end
      if (vif.mon_cb.d_stop === 1'b1) begin
        i2c_ip_side_item it = i2c_ip_side_item::type_id::create("stop");
        it.kind = SIDE_STOP; it.t = $time; n_stop++;
        ap.write(it);
      end
      if ((vif.mon_cb.d_ctrl_active === 1'b1) != act_p) begin
        act_p = (vif.mon_cb.d_ctrl_active === 1'b1);
        h_t.push_back($realtime); h_v.push_back(act_p);
        if (h_t.size() > 4096) begin void'(h_t.pop_front()); void'(h_v.pop_front()); end
      end
      if (vif.mon_cb.d_tgt_hold === 1'b1)  seen_tgt_hold  = 1;
      if (vif.mon_cb.d_ctrl_hold === 1'b1) seen_ctrl_hold = 1;
      if (vif.mon_cb.hs_mode === 1'b1)     seen_hs        = 1;
      if (vif.mon_cb.cs_en === 1'b1)       seen_cs_en     = 1;
      if (vif.mon_cb.irq_err === 1'b1)     seen_irq_err   = 1;
    end
  endtask
endclass : i2c_ip_side_monitor
