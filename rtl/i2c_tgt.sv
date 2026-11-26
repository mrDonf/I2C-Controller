//-----------------------------------------------------------------------------
// i2c_tgt (spec 4.5)
//   I2C target engine.
//   - 7-bit address recognition (ADDR0/ADDR1), general call, reserved
//     address protection, Device ID (0xF8 / 0xF9 sequence, 4.5.6)
//   - target receiver / transmitter with Target RX FIFO tagging
//   - byte-level clock stretching after the ACK bit (or NOSTRETCH mode)
//   - misplaced START / STOP handling (4.5.10)
//
// Structure
//   1. decode     : address match, Device ID, FIFO entries
//   2. decisions  : one strobe per event (flat AND terms)
//   3. registers  : one always block per group of related registers
//   4. outputs
//
// SDA timing: the desired SDA value (r_want) is applied once SCL has been
// observed low for t_hddat cycles (count = 1 in the scl_fall cycle).
// SCL is pulled exactly while the FSM is in T_HOLD.
// r_cnt counts SCL-low cycles (saturating); while a TX byte waits in T_HOLD
// it counts the TSU_DAT setup instead (1 in the first cycle SDA carries the
// new bit).
//-----------------------------------------------------------------------------
module i2c_tgt
  import apb_i2c_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        en,
  input  logic        soft_rst,
  input  logic        tgt_en,
  // bus
  input  logic        scl_f,
  input  logic        sda_f,
  input  logic        scl_rise,
  input  logic        scl_fall,
  input  logic        start_det,
  input  logic        stop_det,
  input  logic        bus_busy,
  input  logic        scl_tout,
  input  logic        ctrl_owner,
  // configuration
  input  logic [6:0]  addr0,
  input  logic        addr0_en,
  input  logic [6:0]  addr1,
  input  logic        addr1_en,
  input  logic        gc_en,
  input  logic        nostretch,
  input  logic        tnack,
  input  logic        devid_en,
  input  logic [23:0] devid,
  input  logic [15:0] t_hddat,   // 0 behaves as 1
  input  logic [15:0] t_sudat,   // 0 behaves as 1
  // target RX FIFO
  input  logic        trx_free1,
  input  logic        trx_free2,
  output logic        trx_push,
  output logic [9:0]  trx_wdata,
  // target TX FIFO
  input  logic        ttx_empty,
  input  logic [7:0]  ttx_rdata,
  output logic        ttx_pop,
  // bus drive
  output logic        scl_oe,
  output logic        sda_oe,
  // events
  output logic        ev_tx_req,
  output logic        ev_taddr,
  output logic        ev_tstop,
  output logic        ev_ttx_done,
  output logic        ev_gc_swrst,
  output logic        ev_trx_ovf,
  output logic        ev_ttx_unf,
  // status
  output logic        active,
  output logic        rw,
  output logic        hold,
  output logic [10:0] tstat
);

  typedef enum logic [2:0] {
    M_RX   = 3'd0,   // target receiver
    M_TX   = 3'd1,   // target transmitter
    M_DEVA = 3'd2,   // Device ID: receiving the target address byte
    M_DEVW = 3'd3,   // Device ID: armed, waiting for Sr
    M_DEVT = 3'd4    // Device ID: transmitting
  } tgt_mode_t;

  //---------------------------------------------------------------------------
  // Registers
  //---------------------------------------------------------------------------
  tgt_state_t  r_state,       state_nxt;
  logic [15:0] r_cnt,         cnt_nxt;        // SCL-low cycles / TSU_DAT count
  tgt_mode_t   r_mode,        mode_nxt;
  logic [1:0]  r_devid_idx,   devid_idx_nxt;
  logic [3:0]  r_bit_cnt,     bit_cnt_nxt;
  logic [7:0]  r_byte,        byte_nxt;       // shift register (rx and tx)
  logic        r_want,        want_nxt;       // desired SDA pull-down
  logic        r_sda_oe,      sda_oe_nxt;
  logic        r_hold_valid,  hold_valid_nxt; // TRX entry (r_byte) not pushed yet
  logic        r_tx_loaded,   tx_loaded_nxt;  // TX byte loaded during stretch
  logic        r_tx_req_sent, tx_req_sent_nxt;
  // transaction status
  logic        r_is_sr;                       // address follows a repeated START
  logic        r_addr_seen,   addr_seen_nxt;  // real address byte in this transaction
  logic        r_first_data,  first_data_nxt; // no data byte accepted since the address
  logic        r_addressed,   addressed_nxt;  // addressed in this transaction
  logic        r_active,      active_nxt;
  logic        r_devid_armed, devid_armed_nxt;
  // configuration latched at START
  logic        r_cfg_en;
  logic [6:0]  r_cfg_addr0;
  logic        r_cfg_addr0_en;
  logic [6:0]  r_cfg_addr1;
  logic        r_cfg_addr1_en;
  logic        r_cfg_gc_en;
  logic        r_cfg_devid_en;
  // TSTAT (r_rw also drives STATUS.TGT_RW, r_gc the GC_SWRST detection)
  logic [6:0]  r_match_addr;
  logic        r_rw;
  logic        r_gc;
  logic        r_addr1_hit;

  //===========================================================================
  // 1. Decode
  //===========================================================================
  logic st_addr, st_rx, st_rxack, st_tx, st_txack, st_hold;
  assign st_addr  = (r_state == T_ADDR);
  assign st_rx    = (r_state == T_RX);
  assign st_rxack = (r_state == T_RXACK);
  assign st_tx    = (r_state == T_TX);
  assign st_txack = (r_state == T_TXACK);
  assign st_hold  = (r_state == T_HOLD);

  logic       mode_rx, mode_deva, mode_devw, mode_devt;
  assign mode_rx   = (r_mode == M_RX);
  assign mode_deva = (r_mode == M_DEVA);
  assign mode_devw = (r_mode == M_DEVW);
  assign mode_devt = (r_mode == M_DEVT);

  // address byte (r_byte = {A[6:0], R/W})
  logic [6:0] rx_addr;
  logic       rsvd_addr;
  logic       hit_addr0, hit_addr1, hit_gc, hit_devid_wr, hit_devid_rd;
  assign rx_addr      = r_byte[7:1];
  assign rsvd_addr    = (rx_addr[6:3] == 4'b0000) | (rx_addr[6:3] == 4'b1111);
  assign hit_addr0    = r_cfg_addr0_en & (rx_addr == r_cfg_addr0) & ~rsvd_addr;
  assign hit_addr1    = r_cfg_addr1_en & (rx_addr == r_cfg_addr1) & ~rsvd_addr;
  assign hit_gc       = r_cfg_gc_en    & (r_byte == 8'h00);
  assign hit_devid_wr = r_cfg_devid_en & (r_byte == 8'hF8);                   // D1
  assign hit_devid_rd = r_cfg_devid_en & r_devid_armed & (r_byte == 8'hF9);   // D7, D9

  logic [7:0] devid_byte;
  always_comb begin : p_devid_byte
    unique case (r_devid_idx)
      2'd0:    devid_byte = devid[23:16];
      2'd1:    devid_byte = devid[15:8];
      default: devid_byte = devid[7:0];
    endcase
  end

  // TRX entries; r_byte is not modified while an entry is held: the held
  // entry is the address if no data byte was accepted after it
  logic [9:0] addr_entry, data_entry, held_entry;
  assign addr_entry = {r_is_sr, 1'b1, r_byte};             // TAG_START / TAG_RSTART
  assign data_entry = {TAG_DATA, r_byte};
  assign held_entry = r_first_data ? addr_entry : data_entry;

  logic [15:0] low_cnt;          // SCL-low count (1 in the scl_fall cycle)
  logic        hd_ok;            // SDA may change: SCL low for THD_DAT cycles
  logic        tx_setup;         // stretching, TX bit on SDA: counting TSU_DAT
  logic        byte_end;         // SCL fall after the 8th bit
  assign low_cnt  = scl_fall ? 16'd1 : r_cnt;
  assign hd_ok    = ~scl_f & (low_cnt >= t_hddat);
  assign tx_setup = st_hold & r_tx_loaded & (r_sda_oe == r_want);
  assign byte_end = scl_fall & (r_bit_cnt == 4'd8);

  //===========================================================================
  // 2. Decisions (priority: EN / SCL_TOUT, STOP, START, bit events)
  //===========================================================================
  logic g_rel, g_stop, g_start, release_bus, run, new_txn;
  assign g_rel       = ~en | scl_tout;
  assign g_stop      = ~g_rel & stop_det;
  assign g_start     = ~g_rel & ~stop_det & start_det;
  assign release_bus = g_rel | g_stop | g_start;
  assign run         = ~release_bus;
  assign new_txn     = g_start & ~bus_busy;        // START from idle (not Sr)

  // shift in SDA on every SCL rising edge of a byte (T_TXACK: ACK bit)
  logic shift_en;
  assign shift_en = run & scl_rise & (st_addr | st_rx | st_tx | st_txack);

  // T_ADDR: address byte complete
  logic a_end, a_blocked, a_devt, a_deva, a_hit, a_ovf, a_ack, a_inactive;
  assign a_end      = run & st_addr & byte_end;
  assign a_blocked  = ~r_cfg_en | ctrl_owner;
  assign a_devt     = a_end & ~a_blocked & hit_devid_rd;
  assign a_deva     = a_end & ~a_blocked & ~hit_devid_rd & hit_devid_wr;
  assign a_hit      = a_end & ~a_blocked & ~hit_devid_rd & ~hit_devid_wr &
                      (hit_addr0 | hit_addr1 | hit_gc);
  assign a_ovf      = a_hit &  nostretch & ~trx_free2;
  assign a_ack      = a_hit & ~(nostretch & ~trx_free2);
  assign a_inactive = a_end & ~a_devt & ~a_deva & ~a_ack;

  // T_RX: data byte (or Device ID target address) complete
  logic r_end, r_deva_ack, r_data, r_ovf;
  assign r_end      = run & st_rx & byte_end;
  assign r_deva_ack = r_end & mode_deva & (hit_addr0 | hit_addr1);           // D3
  assign r_data     = r_end & ~mode_deva & ~tnack &  (trx_free2 | ~nostretch);
  assign r_ovf      = r_end & ~mode_deva & ~tnack & ~(trx_free2 | ~nostretch);

  // ACK clocks and TX bits (SCL falling edge)
  logic rxack_fall, tx_fall, tx_last, txack_fall, tx_more;
  assign rxack_fall = run & st_rxack & scl_fall;
  assign tx_fall    = run & st_tx    & scl_fall;
  assign tx_last    = tx_fall & (r_bit_cnt == 4'd8);
  assign txack_fall = run & st_txack & scl_fall;
  assign tx_more    = txack_fall & ~r_byte[0];             // controller ACK

  // T_HOLD: clock stretching
  logic h_push, h_rx_go, h_tx, h_load, h_req, h_go;
  assign h_push  = run & st_hold &  r_hold_valid & trx_free2;
  assign h_rx_go = run & st_hold & ~r_hold_valid &  mode_rx & (r_sda_oe == r_want);
  assign h_tx    = run & st_hold & ~r_hold_valid & ~mode_rx;
  assign h_load  = h_tx & ~r_tx_loaded & ~ttx_empty;
  assign h_req   = h_tx & ~r_tx_loaded &  ttx_empty & ~r_tx_req_sent;
  assign h_go    = h_tx & tx_setup & (r_cnt >= t_sudat);

  // load the next byte to transmit (SCL fall after an ACK)
  logic do_load, l_devt, l_hold, l_ttx, l_unf, l_req;
  assign do_load = (rxack_fall & ~mode_rx & ~mode_deva & ~mode_devw) | tx_more;
  assign l_devt  = do_load &  mode_devt;
  assign l_hold  = do_load & ~mode_devt &  r_hold_valid;
  assign l_ttx   = do_load & ~mode_devt & ~r_hold_valid & ~ttx_empty;
  assign l_unf   = do_load & ~mode_devt & ~r_hold_valid &  ttx_empty &  nostretch;
  assign l_req   = do_load & ~mode_devt & ~r_hold_valid &  ttx_empty & ~nostretch;

  //===========================================================================
  // 3. Registers
  //===========================================================================
  //---------------------------------------------------------------------------
  // FSM state
  //---------------------------------------------------------------------------
  always_comb begin : p_state_nxt
    state_nxt = r_state;
    if (g_rel || g_stop) begin
      state_nxt = T_IDLE;
    end else if (g_start) begin
      state_nxt = T_ADDR;
    end else begin
      unique case (r_state)
        T_ADDR:
          if (a_end) begin
            if (a_devt || a_deva || a_ack) state_nxt = T_RXACK;
            else                           state_nxt = T_WAIT;    // NACK
          end
        T_RX:
          if (r_end) begin
            if (r_deva_ack || r_data)      state_nxt = T_RXACK;
            else                           state_nxt = T_WAIT;    // NACK
          end
        T_RXACK:
          if (rxack_fall) begin
            if (mode_rx && r_hold_valid)   state_nxt = T_HOLD;    // entry not pushed: stretch
            else if (mode_rx || mode_deva) state_nxt = T_RX;
            else if (mode_devw)            state_nxt = T_WAIT;
            else if (l_hold || l_req)      state_nxt = T_HOLD;    // transmitter: stretch
            else                           state_nxt = T_TX;      // next TX byte loaded
          end
        T_TX:
          if (tx_last)                     state_nxt = T_TXACK;
        T_TXACK:
          if (txack_fall) begin
            if (r_byte[0])                 state_nxt = T_WAIT;    // controller NACK
            else if (l_hold || l_req)      state_nxt = T_HOLD;    // stretch
            else                           state_nxt = T_TX;      // next TX byte loaded
          end
        T_HOLD:
          if (h_rx_go)                     state_nxt = T_RX;
          else if (h_go)                   state_nxt = T_TX;
        default: ;                                                // T_IDLE, T_WAIT
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_state
    if (!rst_n) r_state <= T_IDLE;
    else        r_state <= state_nxt;
  end

  //---------------------------------------------------------------------------
  // Counter: SCL-low cycles, TSU_DAT while a TX byte waits in T_HOLD
  //---------------------------------------------------------------------------
  logic tsu_start;    // TX bit now on SDA: start counting TSU_DAT
  assign tsu_start = run & st_hold & (h_load | r_tx_loaded) & ~tx_setup & (sda_oe_nxt == want_nxt);

  always_comb begin : p_cnt_nxt
    if (scl_fall)                          cnt_nxt = 16'd2;
    else if (tsu_start)                    cnt_nxt = 16'd1;
    else if (~scl_f & (r_cnt != 16'hFFFF)) cnt_nxt = r_cnt + 16'd1;
    else                                   cnt_nxt = r_cnt;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_cnt
    if (!rst_n) r_cnt <= 16'd0;
    else        r_cnt <= cnt_nxt;
  end

  //---------------------------------------------------------------------------
  // Byte engine: bit counter and shift / load register
  //---------------------------------------------------------------------------
  always_comb begin : p_bit_cnt_nxt
    if (g_start | do_load | h_rx_go | h_go | (rxack_fall & ((mode_rx & ~r_hold_valid) | mode_deva)))
      bit_cnt_nxt = 4'd0;
    else if (shift_en & ~st_txack)
      bit_cnt_nxt = r_bit_cnt + 4'd1;
    else
      bit_cnt_nxt = r_bit_cnt;
  end

  always_comb begin : p_byte_nxt
    unique case (1'b1)
      shift_en:       byte_nxt = {r_byte[6:0], sda_f};
      h_load, l_ttx:  byte_nxt = ttx_rdata;
      l_devt:         byte_nxt = devid_byte;
      l_unf:          byte_nxt = 8'hFF;
      default:        byte_nxt = r_byte;
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_byte_engine
    if (!rst_n) begin
      r_bit_cnt <= 4'd0;
      r_byte    <= 8'd0;
    end else begin
      r_bit_cnt <= bit_cnt_nxt;
      r_byte    <= byte_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Mode (receiver / transmitter / Device ID) and Device ID byte index
  //---------------------------------------------------------------------------
  always_comb begin : p_mode_nxt
    unique case (1'b1)
      a_devt:     mode_nxt = M_DEVT;
      a_deva:     mode_nxt = M_DEVA;
      a_ack:      mode_nxt = r_byte[0] ? M_TX : M_RX;
      r_deva_ack: mode_nxt = M_DEVW;
      default:    mode_nxt = r_mode;
    endcase
  end

  always_comb begin : p_devid_idx_nxt
    if (a_devt)      devid_idx_nxt = 2'd0;
    else if (l_devt) devid_idx_nxt = (r_devid_idx == 2'd2) ? 2'd0 : (r_devid_idx + 2'd1);
    else             devid_idx_nxt = r_devid_idx;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_mode
    if (!rst_n) begin
      r_mode      <= M_RX;
      r_devid_idx <= 2'd0;
    end else begin
      r_mode      <= mode_nxt;
      r_devid_idx <= devid_idx_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // SDA drive: desired value, applied THD_DAT after the SCL fall
  //---------------------------------------------------------------------------
  always_comb begin : p_want_nxt
    unique case (1'b1)
      release_bus:                           want_nxt = 1'b0;
      a_devt, a_deva, a_ack,
      r_deva_ack, r_data:                    want_nxt = 1'b1;      // ACK
      tx_fall & ~tx_last:                    want_nxt = ~r_byte[7]; // next bit
      h_load, l_ttx:                         want_nxt = ~ttx_rdata[7];
      l_devt:                                want_nxt = ~devid_byte[7];
      rxack_fall & ~do_load, tx_last,
      txack_fall & ~tx_more,
      l_hold, l_unf, l_req:                  want_nxt = 1'b0;
      default:                               want_nxt = r_want;
    endcase
  end

  always_comb begin : p_sda_oe_nxt
    if (release_bus)      sda_oe_nxt = 1'b0;
    else if (hd_ok)       sda_oe_nxt = want_nxt;
    else                  sda_oe_nxt = r_sda_oe;
  end

  always_ff @(posedge clk or negedge rst_n) begin : p_sda
    if (!rst_n) begin
      r_want   <= 1'b0;
      r_sda_oe <= 1'b0;
    end else begin
      r_want   <= want_nxt;
      r_sda_oe <= sda_oe_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Stretch handshakes: held TRX entry, TX byte loaded, TX_REQ reported
  //---------------------------------------------------------------------------
  assign hold_valid_nxt  = (release_bus | h_push)                ? 1'b0 :
                           ((a_ack | r_data) & ~trx_free2)       ? 1'b1 : r_hold_valid;
  assign tx_loaded_nxt   = (release_bus | h_go | do_load)        ? 1'b0 :
                           h_load                                ? 1'b1 : r_tx_loaded;
  assign tx_req_sent_nxt = (release_bus | (do_load & ~l_req))    ? 1'b0 :
                           (h_req | l_req)                       ? 1'b1 : r_tx_req_sent;

  always_ff @(posedge clk or negedge rst_n) begin : p_stretch
    if (!rst_n) begin
      r_hold_valid  <= 1'b0;
      r_tx_loaded   <= 1'b0;
      r_tx_req_sent <= 1'b0;
    end else begin
      r_hold_valid  <= hold_valid_nxt;
      r_tx_loaded   <= tx_loaded_nxt;
      r_tx_req_sent <= tx_req_sent_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Transaction status
  //---------------------------------------------------------------------------
  logic txn_clr;      // end of the transaction: disable, timeout, STOP, START from idle
  assign txn_clr = g_rel | g_stop | new_txn;

  // real address byte: not the START byte and not an Hs controller code
  assign addr_seen_nxt   = txn_clr ? 1'b0 :
                           a_end   ? ((r_byte != 8'h01) & (r_byte[7:3] != 5'b00001)) : r_addr_seen;
  assign addressed_nxt   = txn_clr ? 1'b0 : a_ack ? 1'b1 : r_addressed;
  assign active_nxt      = (txn_clr | a_inactive) ? 1'b0 : a_ack ? 1'b1 : r_active;
  assign devid_armed_nxt = (txn_clr | (a_end & ~hit_devid_rd)) ? 1'b0 :
                           r_deva_ack                          ? 1'b1 : r_devid_armed;
  assign first_data_nxt  = a_ack ? 1'b1 : r_data ? 1'b0 : r_first_data;

  always_ff @(posedge clk or negedge rst_n) begin : p_txn
    if (!rst_n) begin
      r_is_sr       <= 1'b0;
      r_addr_seen   <= 1'b0;
      r_addressed   <= 1'b0;
      r_active      <= 1'b0;
      r_devid_armed <= 1'b0;
      r_first_data  <= 1'b0;
    end else begin
      // Sr tag only if a real address byte (not START byte / Hs controller
      // code) was already transferred in this transaction
      if (g_start) r_is_sr <= bus_busy & r_addr_seen;
      r_addr_seen   <= addr_seen_nxt;
      r_addressed   <= addressed_nxt;
      r_active      <= active_nxt;
      r_devid_armed <= devid_armed_nxt;
      r_first_data  <= first_data_nxt;
    end
  end

  //---------------------------------------------------------------------------
  // Configuration latched at every START / Sr
  //---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin : p_cfg
    if (!rst_n) begin
      r_cfg_en       <= 1'b0;
      r_cfg_addr0    <= 7'd0;
      r_cfg_addr0_en <= 1'b0;
      r_cfg_addr1    <= 7'd0;
      r_cfg_addr1_en <= 1'b0;
      r_cfg_gc_en    <= 1'b0;
      r_cfg_devid_en <= 1'b0;
    end else if (g_start) begin
      r_cfg_en       <= tgt_en;
      r_cfg_addr0    <= addr0;
      r_cfg_addr0_en <= addr0_en;
      r_cfg_addr1    <= addr1;
      r_cfg_addr1_en <= addr1_en;
      r_cfg_gc_en    <= gc_en;
      r_cfg_devid_en <= devid_en;
    end
  end

  //---------------------------------------------------------------------------
  // TSTAT: updated at every ACKed address, cleared by software reset
  //---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin : p_tstat
    if (!rst_n) begin
      r_match_addr <= 7'd0;
      r_rw         <= 1'b0;
      r_gc         <= 1'b0;
      r_addr1_hit  <= 1'b0;
    end else if (soft_rst) begin
      r_match_addr <= 7'd0;
      r_rw         <= 1'b0;
      r_gc         <= 1'b0;
      r_addr1_hit  <= 1'b0;
    end else if (a_ack) begin
      r_match_addr <= rx_addr;
      r_rw         <= r_byte[0];
      r_gc         <= hit_gc;
      r_addr1_hit  <= hit_addr1 & ~hit_addr0;
    end
  end

  //===========================================================================
  // 4. Outputs
  //===========================================================================
  // Target RX FIFO: an address / data entry needs 2 free entries (room for
  // the STOP entry), the STOP entry 1
  logic stop_entry;
  assign stop_entry = g_stop & r_addressed;

  assign trx_push = (stop_entry & trx_free1) | ((a_ack | r_data) & trx_free2) | h_push;

  always_comb begin : p_trx_wdata
    unique case (1'b1)
      stop_entry: trx_wdata = {TAG_STOP, 8'h00};
      a_ack:      trx_wdata = addr_entry;
      r_data:     trx_wdata = data_entry;
      default:    trx_wdata = held_entry;
    endcase
  end

  assign ttx_pop = h_load | l_ttx;

  // events
  assign ev_tx_req   = h_req | l_req;
  assign ev_taddr    = a_ack;
  assign ev_tstop    = stop_entry;
  assign ev_ttx_done = txack_fall & ~tx_more & ~mode_devt;
  assign ev_gc_swrst = r_data & r_gc & r_first_data & (r_byte == 8'h06);
  assign ev_trx_ovf  = (stop_entry & ~trx_free1) | a_ovf | r_ovf;
  assign ev_ttx_unf  = l_unf;

  // bus drive and status
  assign scl_oe = st_hold;
  assign sda_oe = r_sda_oe;
  assign active = r_active;
  assign rw     = r_rw;
  assign hold   = st_hold;
  assign tstat  = {r_devid_armed, r_addr1_hit, r_gc, r_rw, r_match_addr};

endmodule
