// -----------------------------------------------------------------------------
// i2c_fifo_rx_test  [VP-FIFO-03/04/05]
//   RX FIFOs: empty reads, CRX / TRX thresholds with real traffic and random levels, irq lines (3.7, 4.7, 4.8)
// -----------------------------------------------------------------------------
class i2c_fifo_rx_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_fifo_rx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_fifo_rx_empty_vseq emp = i2c_fifo_rx_empty_vseq::type_id::create("emp");
    i2c_fifo_crx_thr_vseq  crx = i2c_fifo_crx_thr_vseq::type_id::create("crx");
    i2c_fifo_trx_thr_vseq  trx = i2c_fifo_trx_thr_vseq::type_id::create("trx");
    start_seq(emp);
    start_seq(crx);                        // default: 3 bytes, threshold 2
    start_seq(trx);
    repeat (3) begin
      if (!crx.randomize() with { n inside {[1:local::depth]}; thr < local::depth; }) `uvm_fatal("TEST", "randomize")
      start_seq(crx);
      if (!trx.randomize() with { n + 2 <= local::depth; thr < local::depth; }) `uvm_fatal("TEST", "randomize")
      start_seq(trx);
    end
  endtask
endclass : i2c_fifo_rx_test
