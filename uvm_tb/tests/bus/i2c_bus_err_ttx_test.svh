// -----------------------------------------------------------------------------
// i2c_bus_err_ttx_test  [VP-BUS-18]
//   corner: misplaced START / STOP while the target transmits a 1 bit and in the ACK clock of the controller: per-
//   state behaviour, TTX / TRX / events (4.3.2, 4.5.10)
// -----------------------------------------------------------------------------
class i2c_bus_err_ttx_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_bus_err_ttx_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_bus_err_ttx_vseq s = i2c_bus_err_ttx_vseq::type_id::create("ttx");
    i2c_ip_ext_xfer_vseq e = i2c_ip_ext_xfer_vseq::type_id::create("e");
    if (!s.randomize() with { kind == MS_TX_START; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == MS_TX_STOP; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == MS_ACK_START; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    if (!s.randomize() with { kind == MS_ACK_STOP; }) `uvm_fatal("TEST", "randomize")
    start_seq(s);
    repeat (3) rand_start(s);                // random kind and bit position
    if (!e.randomize() with { kind == EX_WRITE; dst == DST_OWN0; data.size() <= local::depth - 2; }) `uvm_fatal("TEST", "randomize")
    e.exp_ack = 1;
    start_seq(e);                            // the target answers normally afterwards
    begin i2c_ip_trx_collect_vseq c = i2c_ip_trx_collect_vseq::type_id::create("c"); c.stop_req = 1; start_seq(c); end
  endtask
endclass : i2c_bus_err_ttx_test
