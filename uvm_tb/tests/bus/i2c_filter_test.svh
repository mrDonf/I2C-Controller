// -----------------------------------------------------------------------------
// i2c_filter_test  [VP-BUS-08]
//   spike filter: SPKLEN_FS boundary (EN = 1 / EN = 0, random length), latency, spikes during a transfer (4.2)
// -----------------------------------------------------------------------------
class i2c_filter_test extends i2c_tgt_base_test;
  `uvm_component_utils(i2c_filter_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_filter_spk_vseq     spk = i2c_filter_spk_vseq::type_id::create("spk");
    i2c_filter_traffic_vseq tr  = i2c_filter_traffic_vseq::type_id::create("tr");
    spk.en = 1;
    start_seq(spk);                        // SPKLEN_FS = 5
    spk.en = 0;
    repeat (2) rand_start(spk);            // random SPKLEN_FS with EN = 0
    configure_dut();                       // target enabled again for the traffic part
    start_seq(tr);
  endtask
endclass : i2c_filter_test
