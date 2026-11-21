// -----------------------------------------------------------------------------
// Layer 1 - bus stimuli of the bit-bang agent (second controller, faulty / slow device)
//   i2c_ip_bb_arb_vseq          second controller: (joined) START + bytes; reports the arbitration result (4.4.6)
//   i2c_ip_bb_scl_stretch_vseq  device holding SCL low inside bytes (bit-level clock stretching, F6 / F7)
//   i2c_ip_bb_glitch_vseq       pulses on SCL / SDA at chosen SCL rises (spikes, misplaced START / STOP)
//   i2c_ip_bb_hold_vseq         line stuck low for a time (0 = until BB_RELEASE)
//   i2c_ip_bb_start_pulse_vseq  SCL pulse inside the hold time of the n-th START / Sr (clock synchronization, 4.4.3)
//   i2c_ip_bb_stop_block_vseq   SDA held low across a STOP of the DUT, then an SCL pulse (STOP vs data bit, 4.4.3)
// -----------------------------------------------------------------------------

class i2c_ip_bb_arb_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_arb_vseq)

  bit [7:0] bytes[$];               // address byte first
  bit       join_start = 1;         // START synchronised with the START of the DUT (BB_JOIN_START)
  bit       end_stop   = 1;         // winner ends with STOP
  bit       lost;
  int       lost_at = -1;           // byte index of the loss
  bit       acks[$];

  function new(string name = "i2c_ip_bb_arb_vseq"); super.new(name); endfunction

  task main();
    lost = 0; lost_at = -1; acks.delete();
    bb(join_start ? BB_JOIN_START : BB_START);
    foreach (bytes[i]) begin
      bit ack, l;
      p_sequencer.bbs.tx(bytes[i], ack, l);
      acks.push_back(ack);
      if (l) begin lost = 1; lost_at = i; break; end
    end
    if (lost) bb(BB_RELEASE);
    else if (end_stop) bb(BB_STOP);
  endtask
endclass : i2c_ip_bb_arb_vseq

class i2c_ip_bb_scl_stretch_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_scl_stretch_vseq)

  rand int unsigned n_holds = 4;
  rand int unsigned first_rise = 3; // SCL rises after the START before the first hold
  rand int unsigned rise_step = 5;  // additional rises between two holds
  rand int unsigned hold_ns = 4000;
  constraint c_def { soft n_holds == 4; soft first_rise == 3; soft rise_step == 5; soft hold_ns == 4000;
                     n_holds inside {[1:16]}; first_rise inside {[1:40]}; rise_step inside {[1:20]}; }

  function new(string name = "i2c_ip_bb_scl_stretch_vseq"); super.new(name); endfunction

  task main();
    bb(BB_WAIT_START);
    for (int i = 0; i < n_holds; i++) begin
      bb(BB_WAIT_SCL_RISE, (i == 0) ? first_rise : rise_step);
      bb(BB_WAIT_SCL_LOW);
      bb(BB_HOLD_SCL, 0, hold_ns);
    end
  endtask
endclass : i2c_ip_bb_scl_stretch_vseq

class i2c_ip_bb_glitch_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_glitch_vseq)

  rand bit          sda_line;       // 1: SDA, 0: SCL
  rand int unsigned n = 1;          // number of pulses
  rand int unsigned rise_step = 13; // SCL rises before each pulse
  rand int unsigned delay_ns = 150; // delay after the SCL rise
  rand int unsigned width_ns = 300; // pulse width
  bit               wait_start = 1; // first wait for a START
  constraint c_def { soft n == 1; soft rise_step == 13; soft delay_ns == 150; soft width_ns == 300; n inside {[1:32]}; }

  function new(string name = "i2c_ip_bb_glitch_vseq"); super.new(name); endfunction

  task main();
    if (wait_start) bb(BB_WAIT_START);
    repeat (n) begin
      if (rise_step > 0) bb(BB_WAIT_SCL_RISE, rise_step);
      if (delay_ns > 0) bb(BB_WAIT, 0, delay_ns);
      bb(sda_line ? BB_GLITCH_SDA : BB_GLITCH_SCL, 0, width_ns);
    end
  endtask
endclass : i2c_ip_bb_glitch_vseq

class i2c_ip_bb_hold_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_hold_vseq)

  rand bit          sda_line;
  rand int unsigned dur_ns;         // 0 = until BB_RELEASE
  bit               wait_low = 0;   // SCL: wait for a low phase of an ongoing transfer first
  constraint c_def { soft dur_ns == 0; }

  function new(string name = "i2c_ip_bb_hold_vseq"); super.new(name); endfunction

  task main();
    if (wait_low) bb(BB_WAIT_SCL_LOW);
    bb(sda_line ? BB_HOLD_SDA : BB_HOLD_SCL, 0, dur_ns);
  endtask
endclass : i2c_ip_bb_hold_vseq

class i2c_ip_bb_start_pulse_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_start_pulse_vseq)

  rand int unsigned nth = 1;        // 1 = START, 2 = first repeated START, ...
  rand int unsigned delay_ns = 100; // after the SDA fall of that START (pad)
  rand int unsigned width_ns = 300; // SCL pulse width
  constraint c_def { soft nth == 1; nth inside {[1:4]}; }

  function new(string name = "i2c_ip_bb_start_pulse_vseq"); super.new(name); endfunction

  task main();
    repeat (nth) bb(BB_WAIT_START);
    if (delay_ns > 0) bb(BB_WAIT, 0, delay_ns);
    bb(BB_GLITCH_SCL, 0, width_ns);
  endtask
endclass : i2c_ip_bb_start_pulse_vseq

class i2c_ip_bb_stop_block_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_bb_stop_block_vseq)

  rand int unsigned rise = 19;             // SCL rise of the STOP clock, counted from the START
  rand int unsigned pulse_after_ns = 1000; // SCL pulse this long after that rise
  rand int unsigned width_ns = 300;
  constraint c_def { soft rise == 19; rise inside {[1:64]}; }

  function new(string name = "i2c_ip_bb_stop_block_vseq"); super.new(name); endfunction

  task main();
    bb(BB_WAIT_START);
    bb(BB_WAIT_SCL_RISE, rise);
    bb(BB_HOLD_SDA);                      // SDA held low: the STOP of the DUT cannot complete
    bb(BB_WAIT, 0, pulse_after_ns);
    bb(BB_GLITCH_SCL, 0, width_ns);       // and the clock goes on (another controller sending a 0)
    bb(BB_WAIT_SCL_HIGH);
    bb(BB_WAIT, 0, 500);
    bb(BB_RELEASE);                       // SDA rises with SCL high: STOP, the bus becomes free
  endtask
endclass : i2c_ip_bb_stop_block_vseq
