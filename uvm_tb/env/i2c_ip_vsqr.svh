// -----------------------------------------------------------------------------
// i2c_ip_vsqr - virtual sequencer: handles to every agent sequencer and to the env services used by the vseqs
// -----------------------------------------------------------------------------
class i2c_ip_vsqr extends uvm_sequencer;
  `uvm_component_utils(i2c_ip_vsqr)

  apb_sequencer       apb_sqr;       // CPU model (APB VIP requester)
  i2c_sequencer       i2c_mst_sqr;   // VIP I2C master (external controller)
  i2c_bb_sequencer    bb_sqr;        // bit-bang / fault injection
  i2c_dmac_driver     dmac;          // DMAC model (operations are programmed directly)
  i2c_ip_reg_block    ral;
  i2c_ip_env_cfg      cfg;
  i2c_ip_scoreboard   sb;
  i2c_ip_side_monitor side;
  i2c_agent           i2c_slv[$];    // VIP slaves (memory models)
  i2c_api_seq         ext;           // running API sequence of the VIP master (write / read / write_read)
  i2c_bb_api_seq      bbs;           // running API sequence of the bit-bang agent
  bit                 ext_owns_bus;  // the VIP master ended its last message without STOP (bus still owned)

  function new(string name, uvm_component parent); super.new(name, parent); endfunction
endclass : i2c_ip_vsqr
