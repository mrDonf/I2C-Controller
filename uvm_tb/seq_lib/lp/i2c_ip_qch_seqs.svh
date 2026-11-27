// -----------------------------------------------------------------------------
// Layer 1 - Q-Channel (low-power) sequences (spec 3.8, 4.12, 6.17)
//   i2c_ip_qch_cycle_vseq  one quiescence cycle: request until accepted, stay in Q_STOPPED for hold_ns (QACTIVE must
//                          stay low and the channel stopped while nothing happens), controller-initiated exit
//   i2c_ip_qch_bg_vseq     background LPI controller: random quiescence requests at random intervals while other
//                          sequences run; accepted requests end by an automatic wake-up (activity) or by a
//                          controller exit after a random hold time
// -----------------------------------------------------------------------------

class i2c_ip_qch_cycle_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_qch_cycle_vseq)

  rand int unsigned hold_ns = 2000;
  rand bit          gate_clk = 1;      // 0: the clock controller keeps PCLK running in Q_STOPPED (allowed by LPI)
  bit               chk_quiet = 1;

  constraint c_def { soft hold_ns inside {[0:20000]}; soft gate_clk == 1; }

  function new(string name = "i2c_ip_qch_cycle_vseq"); super.new(name); endfunction

  task main();
    qch_resp_e r;
    qch_wait_run();
    qch_req(r, gate_clk);
    chk(r == QCH_ACCEPT, $sformatf("idle IP: Q-Channel request %s, expected QCH_ACCEPT", r.name()));
    if (r != QCH_ACCEPT) return;
    chk(qch_pins() == 3'b000, $sformatf("Q_STOPPED: {QACCEPTn,QDENY,QACTIVE} = %b, expected 000", qch_pins()));
    #(hold_ns * 1ns);
    if (chk_quiet) begin
      chk(qch_stopped(), "no wake-up while the IP is idle and nothing happens");
      chk(qch_pins() == 3'b000, $sformatf("Q_STOPPED after %0d ns: {QACCEPTn,QDENY,QACTIVE} = %b", hold_ns, qch_pins()));
    end
    qch_exit();
    chk(qch_pins()[2:1] == 2'b10, $sformatf("Q_RUN after exit: {QACCEPTn,QDENY} = %b", qch_pins()[2:1]));
  endtask
endclass : i2c_ip_qch_cycle_vseq

class i2c_ip_qch_bg_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_qch_bg_vseq)

  rand int unsigned gap_min_ns = 500;
  rand int unsigned gap_max_ns = 20000;
  rand int unsigned hold_max_ns = 10000;  // stopped this long at most before a controller exit (if not woken)
  rand bit          ctrl_exit_en = 1;
  bit               stop_req;
  int unsigned      n_acc, n_deny;

  constraint c_def { gap_min_ns <= gap_max_ns; soft gap_min_ns == 500; soft gap_max_ns == 20000;
                     soft hold_max_ns == 10000; }

  function new(string name = "i2c_ip_qch_bg_vseq"); super.new(name); endfunction

  task main();
    qch_resp_e r;
    while (!stop_req) begin
      #($urandom_range(gap_max_ns, gap_min_ns) * 1ns);
      if (stop_req) break;
      if (qch_stopped()) begin qch_exit(); continue; end     // not woken during the gap: controller exit
      qch_wait_run();
      qch_req(r);
      if (r == QCH_DENY) begin n_deny++; continue; end
      n_acc++;
      if (ctrl_exit_en && $urandom_range(1)) begin
        #($urandom_range(hold_max_ns, 0) * 1ns);
        qch_exit();                                          // no-op when already woken by activity
      end
    end
    qch_exit();
    qch_wait_run(1ms);
    `uvm_info("QCH_BG", $sformatf("background Q-Channel: %0d accepted, %0d denied", n_acc, n_deny), UVM_LOW)
  endtask
endclass : i2c_ip_qch_bg_vseq
