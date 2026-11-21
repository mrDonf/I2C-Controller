// -----------------------------------------------------------------------------
// i2c_bb_pkg - I2C bit-bang / fault-injection agent
//
//   Complements the I2C VIP (which has no multi-controller stimulus, no Hs-mode and no protocol abuse) with
//   primitive bus operations executed by an open-drain driver (i2c_bb_if) on the same SCL/SDA nets:
//     BB_START       wait for a free bus, START                    BB_JOIN_START  START together with another controller
//     BB_RSTART      repeated START (SCL low, owned)               BB_STOP        STOP
//     BB_TX_BYTE     8 bits + ACK sample; wired-AND clock synchronization and arbitration detection (UM10204 3.1.7/3.1.8)
//     BB_RX_BYTE     8 bits sampled, then ACK/NACK
//     BB_HOLD_SCL / BB_HOLD_SDA   pull a line low for dur_ns (0 = until BB_RELEASE)
//     BB_GLITCH_SCL / BB_GLITCH_SDA  pulse a line low for dur_ns (spike)
//     BB_RELEASE     release both lines                           BB_WAIT        wait dur_ns
//     BB_WAIT_START  wait for a START/Sr on the bus               BB_WAIT_SCL_RISE wait `data` SCL rising edges
//     BB_WAIT_SCL_HIGH / BB_WAIT_SCL_LOW  wait for the level      BB_SDA_LOW_CLOCKS hold SDA low for `data` SCL clocks
//     BB_RX_BITS     clock `data` bits with SDA released (no ACK clock); sampled bits returned LSB-aligned in data
//   Timing (cfg): t_low / t_high / t_hd_dat / t_su_sta / t_hd_sta / t_su_sto / t_buf in ns.
// -----------------------------------------------------------------------------
`ifndef I2C_BB_PKG_SV
`define I2C_BB_PKG_SV

`include "uvm_macros.svh"

package i2c_bb_pkg;
  import uvm_pkg::*;
  import i2c_pkg::*;

  typedef virtual i2c_bb_if bb_vif_t;

  typedef enum { BB_START, BB_JOIN_START, BB_RSTART, BB_STOP, BB_TX_BYTE, BB_RX_BYTE,
                 BB_HOLD_SCL, BB_HOLD_SDA, BB_GLITCH_SCL, BB_GLITCH_SDA, BB_RELEASE, BB_WAIT,
                 BB_WAIT_START, BB_WAIT_SCL_RISE, BB_WAIT_SCL_HIGH, BB_WAIT_SCL_LOW, BB_SDA_LOW_CLOCKS,
                 BB_RX_BITS } bb_op_e;

  class i2c_bb_cfg extends uvm_object;
    `uvm_object_utils(i2c_bb_cfg)
    bb_vif_t  vif;
    i2c_vif_t bus;                        // bus levels (VIP interface nets)
    real t_low_ns    = 1500.0;
    real t_high_ns   = 700.0;
    real t_hd_dat_ns = 300.0;
    real t_su_sta_ns = 700.0;
    real t_hd_sta_ns = 700.0;
    real t_su_sto_ns = 700.0;
    real t_buf_ns    = 1500.0;
    function new(string name = "i2c_bb_cfg"); super.new(name); endfunction
    function void set_speed(real low, real high, real hd_dat, real su);
      t_low_ns = low; t_high_ns = high; t_hd_dat_ns = hd_dat;
      t_su_sta_ns = su; t_hd_sta_ns = su; t_su_sto_ns = su; t_buf_ns = low;
    endfunction
  endclass : i2c_bb_cfg

  class i2c_bb_item extends uvm_sequence_item;
    `uvm_object_utils(i2c_bb_item)
    bb_op_e      op;
    bit [7:0]    data;                  // TX byte / RX: returned byte / count for the WAIT_* ops
    bit          ack;                   // TX: ACK seen (1 = ACK); RX: ACK to send
    real         dur_ns;
    bit          arb_lost;              // response
    function new(string name = "i2c_bb_item"); super.new(name); endfunction
    virtual function string convert2string();
      return $sformatf("%s data=0x%02h ack=%0d dur=%0.0f%s", op.name(), data, ack, dur_ns, arb_lost ? " ARB_LOST" : "");
    endfunction
  endclass : i2c_bb_item

  class i2c_bb_sequencer extends uvm_sequencer #(i2c_bb_item);
    `uvm_component_utils(i2c_bb_sequencer)
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
  endclass : i2c_bb_sequencer

  // ---------------------------------------------------------------- driver
  class i2c_bb_driver extends uvm_driver #(i2c_bb_item);
    `uvm_component_utils(i2c_bb_driver)
    i2c_bb_cfg cfg;
    protected bit lost;

    function new(string name, uvm_component parent); super.new(name, parent); endfunction

    task run_phase(uvm_phase phase);
      cfg.vif.scl_oe = 1'b0; cfg.vif.sda_oe = 1'b0;
      forever begin
        seq_item_port.get_next_item(req);
        execute(req);
        seq_item_port.item_done();
      end
    endtask

    protected function logic scl(); return cfg.bus.scl; endfunction
    protected function logic sda(); return cfg.bus.sda; endfunction
    protected task dly(real ns); if (ns > 0) #(ns * 1ns); endtask

    // one clock with wired-AND synchronization; precondition: SCL low (held by us)
    //   drive = value put on SDA (1 = release), check_arb: lost when SDA low while we release it during SCL high
    protected task clock_bit(bit drive, bit check_arb, output bit sampled);
      realtime t0;
      dly(cfg.t_hd_dat_ns);
      cfg.vif.sda_oe = !drive;
      dly(cfg.t_low_ns - cfg.t_hd_dat_ns);
      cfg.vif.scl_oe = 1'b0;
      wait (cfg.bus.scl === 1'b1);                              // other devices may still hold SCL low (stretch / sync)
      t0 = $realtime;
      sampled = 1'b1;
      fork
        begin : hi_time
          dly(cfg.t_high_ns);
        end
        begin : early_fall                                // another controller ends the high phase first
          wait (cfg.bus.scl === 1'b0);
        end
        begin : sample_arb
          #1ns;
          sampled = sda();
          forever begin
            if (check_arb && drive && sda() === 1'b0) begin lost = 1; break; end
            @(cfg.bus.sda or cfg.bus.scl);
            if (scl() !== 1'b1) break;
          end
          wait (0);
        end
      join_any
      disable fork;
      if (check_arb && drive && sampled === 1'b0) lost = 1;
      if (lost) begin
        cfg.vif.sda_oe = 1'b0;
        cfg.vif.scl_oe = 1'b0;
        return;
      end
      cfg.vif.scl_oe = 1'b1;
    endtask

    protected task tx_byte(bit [7:0] v, output bit ack);
      bit s;
      for (int i = 7; i >= 0; i--) begin
        clock_bit(v[i], 1'b1, s);
        if (lost) return;
      end
      clock_bit(1'b1, 1'b0, s);
      ack = (s === 1'b0);
    endtask

    protected task rx_byte(output bit [7:0] v, input bit ack);
      bit s;
      for (int i = 7; i >= 0; i--) begin
        clock_bit(1'b1, 1'b0, s);
        v[i] = s;
      end
      clock_bit(!ack, 1'b1, s);                          // NACK (release) can lose against another receiver's ACK
    endtask

    protected task execute(i2c_bb_item it);
      lost = 0;
      case (it.op)
        BB_START: begin
          while (scl() !== 1'b1 || sda() !== 1'b1) #(50ns);
          dly(cfg.t_buf_ns);
          while (scl() !== 1'b1 || sda() !== 1'b1) #(50ns);
          cfg.vif.sda_oe = 1'b1;
          dly(cfg.t_hd_sta_ns);
          cfg.vif.scl_oe = 1'b1;
        end
        BB_JOIN_START: begin
          do @(negedge cfg.bus.sda); while (scl() !== 1'b1);
          cfg.vif.sda_oe = 1'b1;
          dly(cfg.t_hd_sta_ns);
          cfg.vif.scl_oe = 1'b1;
        end
        BB_RSTART: begin
          dly(cfg.t_hd_dat_ns);
          cfg.vif.sda_oe = 1'b0;
          dly(cfg.t_low_ns - cfg.t_hd_dat_ns);
          cfg.vif.scl_oe = 1'b0;
          wait (cfg.bus.scl === 1'b1);
          dly(cfg.t_su_sta_ns);
          cfg.vif.sda_oe = 1'b1;
          dly(cfg.t_hd_sta_ns);
          cfg.vif.scl_oe = 1'b1;
        end
        BB_STOP: begin
          dly(cfg.t_hd_dat_ns);
          cfg.vif.sda_oe = 1'b1;
          dly(cfg.t_low_ns - cfg.t_hd_dat_ns);
          cfg.vif.scl_oe = 1'b0;
          wait (cfg.bus.scl === 1'b1);
          dly(cfg.t_su_sto_ns);
          cfg.vif.sda_oe = 1'b0;
        end
        BB_TX_BYTE: begin bit a; tx_byte(it.data, a); it.ack = a; end
        BB_RX_BYTE: begin bit [7:0] v; rx_byte(v, it.ack); it.data = v; end
        BB_RX_BITS: begin                                    // part of a byte (misplaced S/P, spec 4.5.10)
          bit [7:0] v;
          bit       s;
          v = 8'h00;
          repeat (it.data) begin clock_bit(1'b1, 1'b0, s); v = {v[6:0], s}; end
          it.data = v;
        end
        BB_HOLD_SCL: begin
          cfg.vif.scl_oe = 1'b1;
          if (it.dur_ns > 0) begin dly(it.dur_ns); cfg.vif.scl_oe = 1'b0; end
        end
        BB_HOLD_SDA: begin
          cfg.vif.sda_oe = 1'b1;
          if (it.dur_ns > 0) begin dly(it.dur_ns); cfg.vif.sda_oe = 1'b0; end
        end
        BB_GLITCH_SCL: begin cfg.vif.scl_oe = 1'b1; dly(it.dur_ns); cfg.vif.scl_oe = 1'b0; end
        BB_GLITCH_SDA: begin cfg.vif.sda_oe = 1'b1; dly(it.dur_ns); cfg.vif.sda_oe = 1'b0; end
        BB_RELEASE: begin cfg.vif.scl_oe = 1'b0; cfg.vif.sda_oe = 1'b0; end
        BB_WAIT: dly(it.dur_ns);
        BB_WAIT_START: begin do @(negedge cfg.bus.sda); while (scl() !== 1'b1); end
        BB_WAIT_SCL_RISE: repeat (it.data) @(posedge cfg.bus.scl);
        BB_WAIT_SCL_HIGH: wait (cfg.bus.scl === 1'b1);
        BB_WAIT_SCL_LOW: wait (cfg.bus.scl === 1'b0);
        BB_SDA_LOW_CLOCKS: begin
          cfg.vif.sda_oe = 1'b1;
          repeat (it.data) @(posedge cfg.bus.scl);
          @(negedge cfg.bus.scl);
          dly(cfg.t_hd_dat_ns);
          cfg.vif.sda_oe = 1'b0;
        end
        default: ;
      endcase
      it.arb_lost = lost;
      `uvm_info("BB", it.convert2string(), UVM_HIGH)
    endtask
  endclass : i2c_bb_driver

  class i2c_bb_agent extends uvm_agent;
    `uvm_component_utils(i2c_bb_agent)
    i2c_bb_cfg       cfg;
    i2c_bb_sequencer sqr;
    i2c_bb_driver    drv;
    function new(string name, uvm_component parent); super.new(name, parent); endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(i2c_bb_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("BB_CFG", "no i2c_bb_cfg")
      sqr = i2c_bb_sequencer::type_id::create("sqr", this);
      drv = i2c_bb_driver::type_id::create("drv", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.cfg = cfg;
      drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass : i2c_bb_agent

  // ---------------------------------------------------------------- API sequence (helpers callable from a vseq)
  class i2c_bb_api_seq extends uvm_sequence #(i2c_bb_item);
    `uvm_object_utils(i2c_bb_api_seq)
    bit stop_seq;
    function new(string name = "i2c_bb_api_seq"); super.new(name); endfunction
    task body(); wait (stop_seq); endtask

    task op(bb_op_e o, bit [7:0] data = 0, bit ack = 0, real dur_ns = 0, output i2c_bb_item rsp);
      i2c_bb_item it = i2c_bb_item::type_id::create("bb");
      it.op = o; it.data = data; it.ack = ack; it.dur_ns = dur_ns;
      start_item(it);
      finish_item(it);
      rsp = it;
    endtask
    task do_op(bb_op_e o, bit [7:0] data = 0, bit ack = 0, real dur_ns = 0);
      i2c_bb_item r;
      op(o, data, ack, dur_ns, r);
    endtask
    task start_c();  do_op(BB_START);  endtask
    task rstart_c(); do_op(BB_RSTART); endtask
    task stop_c();   do_op(BB_STOP);   endtask
    task tx(bit [7:0] v, output bit ack, output bit lost);
      i2c_bb_item r;
      op(BB_TX_BYTE, v, 0, 0, r);
      ack = r.ack; lost = r.arb_lost;
    endtask
    task rx_bits(int unsigned n, output bit [7:0] v);
      i2c_bb_item r;
      op(BB_RX_BITS, 8'(n), 0, 0, r);
      v = r.data;
    endtask
    task rx(output bit [7:0] v, input bit ack);
      i2c_bb_item r;
      op(BB_RX_BYTE, 0, ack, 0, r);
      v = r.data;
    endtask
  endclass : i2c_bb_api_seq

endpackage : i2c_bb_pkg

`endif
