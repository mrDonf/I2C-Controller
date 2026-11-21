// -----------------------------------------------------------------------------
// i2c_ip_env - top environment of apb_i2c
//
//   apb      : APB VIP env, requester agent (CPU model; the DMAC model shares its sequencer)
//   i2c      : I2C VIP env, master agent (external controller) + slave agents (external targets 0x50, 0x51)
//   dma      : DMAC agent (reactive responder on dma_req / dma_ack)
//   bb       : bit-bang / fault-injection agent (second controller, glitches, stuck lines, misplaced S/P)
//   side_mon : sideband / debug port monitor
//   ral      : register model + predictor (APB monitor -> mirror)
//   sb       : spec reference model / scoreboard
//   cov      : functional coverage
//   vsqr     : virtual sequencer
// -----------------------------------------------------------------------------
class i2c_ip_env extends uvm_env;
  `uvm_component_utils(i2c_ip_env)

  i2c_ip_env_cfg      cfg;
  apb_env             apb;
  i2c_env             i2c;
  i2c_dma_agent       dma;
  i2c_bb_agent        bb;
  i2c_ip_side_monitor side_mon;
  i2c_ip_reg_block    ral;
  i2c_ip_reg_adapter  adapter;
  uvm_reg_predictor #(apb_item) predictor;
  i2c_ip_scoreboard   sb;
  i2c_ip_coverage     cov;
  i2c_ip_vsqr         vsqr;

  function new(string name, uvm_component parent); super.new(name, parent); endfunction

  function void build_phase(uvm_phase phase);
    apb_env_cfg a;
    i2c_env_cfg e;
    super.build_phase(phase);
    if (!uvm_config_db #(i2c_ip_env_cfg)::get(this, "", "cfg", cfg)) `uvm_fatal("ENV_CFG", "no i2c_ip_env_cfg")

    a = apb_env_cfg::type_id::create("apb_env_cfg");
    a.mst_cfg = cfg.apb_cfg; a.has_scoreboard = 0;                      // registers are not a memory
    uvm_config_db #(apb_env_cfg)::set(this, "apb", "cfg", a);
    apb = apb_env::type_id::create("apb", this);

    e = i2c_env_cfg::type_id::create("i2c_env_cfg");
    e.mst_cfg = cfg.i2c_mst_cfg; e.slv_cfg = cfg.i2c_slv_cfg; e.has_scoreboard = 0;
    uvm_config_db #(i2c_env_cfg)::set(this, "i2c", "cfg", e);
    i2c = i2c_env::type_id::create("i2c", this);

    uvm_config_db #(i2c_dma_cfg)::set(this, "dma", "cfg", cfg.dma_cfg);
    dma = i2c_dma_agent::type_id::create("dma", this);
    uvm_config_db #(i2c_bb_cfg)::set(this, "bb", "cfg", cfg.bb_cfg);
    bb = i2c_bb_agent::type_id::create("bb", this);

    side_mon = i2c_ip_side_monitor::type_id::create("side_mon", this);

    ral = i2c_ip_reg_block::type_id::create("ral");
    ral.fifo_depth = cfg.p.fifo_depth;
    ral.devid_rst  = cfg.p.devid_rst;
    ral.version    = cfg.p.version;
    ral.build();
    ral.reset();                                                      // mirror = reset values
    adapter   = i2c_ip_reg_adapter::type_id::create("adapter");
    adapter.ral = ral;
    predictor = uvm_reg_predictor #(apb_item)::type_id::create("predictor", this);

    sb = i2c_ip_scoreboard::type_id::create("sb", this);
    if (cfg.has_coverage) cov = i2c_ip_coverage::type_id::create("cov", this);
    vsqr = i2c_ip_vsqr::type_id::create("vsqr", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    side_mon.vif = cfg.side_vif;

    // register layer: frontdoor through the APB requester, mirror from the APB monitor
    ral.default_map.set_sequencer(apb.mst.sqr, adapter);
    ral.default_map.set_auto_predict(0);
    predictor.map     = ral.default_map;
    predictor.adapter = adapter;
    apb.mst.ap.connect(predictor.bus_in);

    // DMAC model: second requester on the APB sequencer
    if (dma.drv != null) dma.drv.apb_sqr = apb.mst.sqr;

    // reference model
    sb.cfg  = cfg;
    sb.side = side_mon;
    apb.mst.ap.connect(sb.apb_imp);
    i2c.mst.ap.connect(sb.bus_imp);
    side_mon.ap.connect(sb.side_imp);

    if (cov != null) begin
      cov.cfg = cfg;
      cov.side_vif = cfg.side_vif;
      apb.mst.ap.connect(cov.apb_imp);
      sb.ap_msg.connect(cov.msg_imp);
      side_mon.ap.connect(cov.side_imp);
      dma.mon.ap.connect(cov.dma_imp);
    end

    vsqr.apb_sqr     = apb.mst.sqr;
    vsqr.i2c_mst_sqr = i2c.mst.sqr;
    vsqr.bb_sqr      = bb.sqr;
    vsqr.dmac        = dma.drv;
    vsqr.ral         = ral;
    vsqr.cfg         = cfg;
    vsqr.sb          = sb;
    vsqr.side        = side_mon;
    vsqr.i2c_slv     = i2c.slv;
  endfunction

  // API sequences that stay alive for the whole test (helpers callable from any virtual sequence)
  task run_phase(uvm_phase phase);
    vsqr.ext = i2c_api_seq::type_id::create("ext");
    vsqr.bbs = i2c_bb_api_seq::type_id::create("bbs");
    fork
      vsqr.ext.start(i2c.mst.sqr);
      vsqr.bbs.start(bb.sqr);
    join_none
  endtask
endclass : i2c_ip_env
