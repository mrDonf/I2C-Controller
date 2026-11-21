// -----------------------------------------------------------------------------
// i2c_ip_tb_params - parameters of the DUT instance (set by tb_top, uvm_config_db key "tb_params")
// i2c_ip_env_cfg   - configuration of the whole environment (uvm_config_db key "cfg" for the env)
// -----------------------------------------------------------------------------
class i2c_ip_tb_params extends uvm_object;
  `uvm_object_utils(i2c_ip_tb_params)
  int unsigned fifo_depth = 16;
  bit [23:0]   devid_rst  = 24'h0;
  bit [31:0]   version    = 32'h0002_0000;
  real         pclk_ns    = 10.0;
  function new(string name = "i2c_ip_tb_params"); super.new(name); endfunction
endclass : i2c_ip_tb_params

class i2c_ip_env_cfg extends uvm_object;
  `uvm_object_utils(i2c_ip_env_cfg)

  i2c_ip_tb_params p;

  // agents
  apb_agent_cfg  apb_cfg;
  i2c_agent_cfg  i2c_mst_cfg;          // VIP master (external controller for the DUT target)
  i2c_agent_cfg  i2c_slv_cfg[$];       // VIP slaves (external targets for the DUT controller)
  i2c_bb_cfg     bb_cfg;
  i2c_dma_cfg    dma_cfg;
  side_vif_t     side_vif;
  virtual vip_clk_rst_if crif;

  // VIP slave addresses (7-bit): an EEPROM model at 0x50 and a second one at 0x51
  bit [6:0]      slv_addr[$] = '{7'h50, 7'h51};
  // own target addresses of the DUT (TADDR.ADDR0 / ADDR1), set by i2c_ip_init_vseq
  bit [6:0]      tgt_addr[2] = '{7'h3A, 7'h2B};

  // checkers / coverage
  bit            has_coverage   = 1'b1;
  bit            sb_ctrl_check  = 1'b1;  // CMD -> bus -> CRX model
  bit            sb_tgt_check   = 1'b1;  // bus -> TRX / TTX -> bus model
  bit            sb_apb_check   = 1'b1;  // APB access rules (PSLVERR, read data of errors)

  i2c_ip_speed_e speed = SPD_FM;          // current F/S speed (coverage, VIP timing)

  function new(string name = "i2c_ip_env_cfg"); super.new(name); endfunction

  // VIP timing for a speed mode: monitor limits of the mode, VIP master clocked at the mode
  function void set_vip_speed(i2c_ip_speed_e s);
    speed = s;
    i2c_mst_cfg.speed = vip_speed(s);
    foreach (i2c_slv_cfg[i]) i2c_slv_cfg[i].speed = vip_speed(s);
    case (s)
      SPD_SM:  begin i2c_mst_cfg.t_hd_dat_ns = 300; foreach (i2c_slv_cfg[i]) i2c_slv_cfg[i].slave_hd_dat_ns = 300; end
      SPD_FM:  begin i2c_mst_cfg.t_hd_dat_ns = 150; foreach (i2c_slv_cfg[i]) i2c_slv_cfg[i].slave_hd_dat_ns = 200; end
      default: begin i2c_mst_cfg.t_hd_dat_ns = 80;  foreach (i2c_slv_cfg[i]) i2c_slv_cfg[i].slave_hd_dat_ns = 100; end
    endcase
    case (s)
      SPD_SM:  bb_cfg.set_speed(5000, 4500, 600, 5000);
      SPD_FM:  bb_cfg.set_speed(1500,  800, 300,  800);
      default: bb_cfg.set_speed( 650,  350, 100,  350);
    endcase
  endfunction

  function bit is_vip_slave(bit [6:0] a);
    foreach (slv_addr[i]) if (slv_addr[i] == a) return 1;
    return 0;
  endfunction
endclass : i2c_ip_env_cfg
