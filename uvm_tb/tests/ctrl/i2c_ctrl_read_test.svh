// -----------------------------------------------------------------------------
// i2c_ctrl_read_test  [VP-CTL-02]
//   controller read (6.4): 1 byte .. longer than CRX, LAST on the final byte, both VIP targets
// -----------------------------------------------------------------------------
class i2c_ctrl_read_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_read_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ctrl_xfer_vseq  x    = i2c_ip_ctrl_xfer_vseq::type_id::create("x");
    i2c_ctrl_idle_chk_vseq idle = i2c_ctrl_idle_chk_vseq::type_id::create("idle");
    if (!x.randomize() with { kind == CX_WRITE; addr == 7'h50; ptr == 8'h00; data.size() == 32; }) `uvm_fatal("TEST", "randomize")
    start_seq(x);
    foreach (cfg.slv_addr[i]) begin
      if (!x.randomize() with { kind == CX_READ; addr == local::cfg.slv_addr[i]; rd_len == 1; }) `uvm_fatal("TEST", "randomize")
      start_seq(x);
    end
    repeat (6) begin
      if (!x.randomize() with { kind == CX_READ; rd_len inside {[1:2 * local::depth + 3]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(x);
    end
    start_seq(idle);
  endtask
endclass : i2c_ctrl_read_test
