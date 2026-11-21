// -----------------------------------------------------------------------------
// i2c_arb_lost_addr_test  [VP-BUS-01]
//   arbitration lost in the address, the winner addresses the DUT target; re-try (4.4.6, 6.13)
// -----------------------------------------------------------------------------
class i2c_arb_lost_addr_test extends i2c_bus_base_test;
  `uvm_component_utils(i2c_arb_lost_addr_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  // the winner must send a 0 where the DUT sends a 1: ADDR0 below 0x51 (random)
  virtual function bit randomize_init(i2c_ip_init_vseq s);
    return s.randomize() with { ctrl_en == 1; tgt_en == 1; tcfg == 0; intr_en == 0; addr0 < 7'h50; };
  endfunction

  virtual task run_scenario();
    i2c_arb_lost_addr_vseq s = i2c_arb_lost_addr_vseq::type_id::create("arb");
    repeat (2) start_seq(s);
  endtask
endclass : i2c_arb_lost_addr_test
