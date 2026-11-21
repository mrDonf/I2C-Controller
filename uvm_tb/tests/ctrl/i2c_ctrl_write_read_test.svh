// -----------------------------------------------------------------------------
// i2c_ctrl_write_read_test  [VP-CTL-03]
//   register write then register read with repeated START (6.5): the read returns the written bytes
// -----------------------------------------------------------------------------
class i2c_ctrl_write_read_test extends i2c_ctrl_base_test;
  `uvm_component_utils(i2c_ctrl_write_read_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_ip_ctrl_xfer_vseq wseq = i2c_ip_ctrl_xfer_vseq::type_id::create("wseq");
    i2c_ip_ctrl_xfer_vseq rseq = i2c_ip_ctrl_xfer_vseq::type_id::create("rseq");
    repeat (8) begin
      if (!wseq.randomize() with { kind == CX_WRITE; data.size() inside {[1:local::depth + 4]}; }) `uvm_fatal("TEST", "randomize")
      start_seq(wseq);
      // the read uses the target / pointer / length chosen for the write
      if (!rseq.randomize() with { kind == CX_WRITE_READ; addr == local::wseq.addr; ptr == local::wseq.ptr; rd_len == local::wseq.data.size(); })
        `uvm_fatal("TEST", "randomize")
      start_seq(rseq);
      foreach (wseq.data[i])
        if (rseq.rdata[i] != wseq.data[i]) `uvm_error("TEST", $sformatf("byte %0d read 0x%02h, written 0x%02h", i, rseq.rdata[i], wseq.data[i]))
    end
  endtask
endclass : i2c_ctrl_write_read_test
