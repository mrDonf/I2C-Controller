// -----------------------------------------------------------------------------
// i2c_reg_access_err_test  [VP-REG-06/07/08]
//   RO / WO registers, unmapped offsets (PSLVERR), PADDR[1:0] and PPROT ignored (3.1, 3.7)
// -----------------------------------------------------------------------------
class i2c_reg_access_err_test extends i2c_reg_base_test;
  `uvm_component_utils(i2c_reg_access_err_test)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  virtual task run_scenario();
    i2c_reg_ro_wo_vseq       rowo = i2c_reg_ro_wo_vseq::type_id::create("rowo");
    i2c_reg_unmapped_vseq    unm  = i2c_reg_unmapped_vseq::type_id::create("unm");
    i2c_reg_paddr_pprot_vseq pp   = i2c_reg_paddr_pprot_vseq::type_id::create("pp");
    start_seq(rowo);
    unm.n_rand = 16;
    start_seq(unm);
    start_seq(pp);
  endtask
endclass : i2c_reg_access_err_test
