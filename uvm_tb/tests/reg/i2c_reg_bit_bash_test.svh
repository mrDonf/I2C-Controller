// -----------------------------------------------------------------------------
// i2c_reg_bit_bash_test  [VP-REG-03]
//   RW registers: UVM uvm_reg_bit_bash_seq + pattern / reserved-bit checks (5.1, 3.7)
// -----------------------------------------------------------------------------
class i2c_reg_bit_bash_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_bit_bash_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    uvm_reg_bit_bash_seq    bash = uvm_reg_bit_bash_seq::type_id::create("bash");
    i2c_reg_rw_pattern_vseq pat  = i2c_reg_rw_pattern_vseq::type_id::create("pat");
    bash.model = env.ral;
    bash.start(null);
    pat.n_rand = 4;
    start_seq(pat);
  endtask
endclass : i2c_reg_bit_bash_test
