// -----------------------------------------------------------------------------
// Layer 1 - traffic sequences (reusable, randomizable; data checked by the scoreboard on every message)
//   i2c_ip_ctrl_xfer_vseq    DUT controller transfer to an external target: write / read / write-read (6.3 - 6.5)
//   i2c_ip_ctrl_cmd_vseq     raw command list with an expected outcome (CMPL / NACK / CMD_ERR ...) (4.4)
//   i2c_ip_ext_xfer_vseq     external controller (VIP master) transfer to the DUT target (or any address)
//   i2c_ip_trx_collect_vseq  background TRX reader (software draining while a transfer runs)
//   i2c_ip_ttx_service_vseq  TX_REQ service routine of 6.11 (TRX drain + TTX refill)
//   i2c_ip_dma_start_vseq    DMAC channel + DMA_BLK / DMA_CTRL programming (6.15)
//   i2c_ip_dma_check_vseq    end of a DMA operation: request sequence (4.9.4), DMA_DONE, DMA_STAT, DMA_CTRL.EN
// -----------------------------------------------------------------------------

typedef enum { CX_WRITE, CX_READ, CX_WRITE_READ } i2c_ip_ctrl_xfer_e;
typedef enum { CMD_EXP_CMPL, CMD_EXP_NACK, CMD_EXP_ERR, CMD_EXP_ERR_IDLE, CMD_EXP_NONE } i2c_ip_cmd_exp_e;
typedef enum { EX_WRITE, EX_READ, EX_WRITE_READ } i2c_ip_ext_xfer_e;
typedef enum { DST_OWN0, DST_OWN1, DST_OTHER } i2c_ip_ext_dst_e;

// ---------------------------------------------------------------- DUT controller transfer
class i2c_ip_ctrl_xfer_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_ctrl_xfer_vseq)

  rand i2c_ip_ctrl_xfer_e kind;
  rand bit [6:0]          addr = 7'h50;
  rand bit [7:0]          ptr;        // register pointer of the EEPROM model (first byte of WRITE / WRITE_READ)
  rand bit [7:0]          data[];     // WRITE payload after the pointer
  rand int unsigned       rd_len = 1; // READ / WRITE_READ length
  rand bit                no_ptr;     // WRITE without pointer byte (S A W d.. P, S A W P when data is empty)
  bit                     chk_mem = 1; // compare with the VIP slave memory
  bit [7:0]               rdata[$];

  constraint c_addr { soft addr inside {7'h50, 7'h51}; }
  constraint c_len  { data.size() <= 96; soft data.size() inside {[1:16]}; rd_len inside {[1:96]}; soft rd_len inside {[1:16]}; }
  constraint c_def  { soft no_ptr == 0; soft kind == CX_WRITE; }

  function new(string name = "i2c_ip_ctrl_xfer_vseq"); super.new(name); endfunction

  function int slv_idx(bit [6:0] a);
    foreach (cfg.slv_addr[i]) if (cfg.slv_addr[i] == a) return i;
    return -1;
  endfunction

  task main();
    bit [7:0] d[$];
    int si = slv_idx(addr);
    `uvm_info("CXFER", $sformatf("%s 0x%02h ptr 0x%02h wr %0d rd %0d", kind.name(), addr, ptr, data.size(), rd_len), UVM_HIGH)
    case (kind)
      CX_WRITE: begin
        foreach (data[i]) d.push_back(data[i]);
        if (!no_ptr) d.push_front(ptr);
        ctrl_write(addr, d);
        if (chk_mem && si >= 0 && !no_ptr)
          foreach (data[i]) chk(slv_mem(si, ptr + i) == data[i],
                                $sformatf("VIP slave 0x%02h mem[0x%02h] = 0x%02h, written 0x%02h", addr, 8'(ptr + i), slv_mem(si, ptr + i), data[i]));
      end
      CX_READ: ctrl_read(addr, rd_len, rdata);
      CX_WRITE_READ: begin
        ctrl_write_read(addr, '{ptr}, rd_len, rdata);
        if (chk_mem && si >= 0)
          foreach (rdata[i]) chk(rdata[i] == slv_mem(si, ptr + i), $sformatf("write-read byte %0d = 0x%02h, VIP slave memory 0x%02h", i, rdata[i], slv_mem(si, ptr + i)));
      end
    endcase
  endtask
endclass : i2c_ip_ctrl_xfer_vseq

// ---------------------------------------------------------------- raw commands + expected outcome
class i2c_ip_ctrl_cmd_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_ctrl_cmd_vseq)

  bit [12:0]       cmds[$];
  i2c_ip_cmd_exp_e exp       = CMD_EXP_CMPL;
  bit [6:0]        exp_cstat = 0;
  int              n_crx     = -1;    // >= 0: CRX entries expected (drained concurrently for CMPL, checked + read otherwise)
  string           what      = "commands";
  realtime         tmo       = 2ms;
  bit [8:0]        crx[$];

  function new(string name = "i2c_ip_ctrl_cmd_vseq"); super.new(name); endfunction

  task main();
    int unsigned s0 = p_sequencer.side.n_start;
    crx.delete();
    if (exp == CMD_EXP_CMPL && n_crx > 0) fork push_cmds(cmds); drain_crx(n_crx, crx); join
    else push_cmds(cmds);
    case (exp)
      CMD_EXP_CMPL: wait_cmpl(exp_cstat, what, tmo);
      CMD_EXP_NACK: begin wait_ev(I_NACK, tmo, 1, what); wait_cmpl(exp_cstat, what, tmo); end
      CMD_EXP_ERR:  begin wait_ev(I_CMD_ERR, tmo, 1, what); wait_cmpl(7'h10, what, tmo); end
      CMD_EXP_ERR_IDLE: begin
        wait_ev(I_CMD_ERR, tmo, 1, what);
        #(100us);
        chk(p_sequencer.side.n_start == s0, {what, ": no bus activity"});
        chk_ev(I_CMPL, 0, {what, ": no CMPL"});
        chk_cstat(7'h10, what);
      end
      default: ;
    endcase
    if (exp inside {CMD_EXP_NACK, CMD_EXP_ERR, CMD_EXP_ERR_IDLE}) begin
      chk_rd(R_FIFO_STAT, 0, 32'h3F, {what, ": CMD FIFO flushed"});
      if (n_crx >= 0) begin
        chk_rd(R_FIFO_STAT, n_crx << 8, 32'h3F << 8, {what, ": CRX holds the real bytes"});
        repeat (n_crx) begin bit [31:0] v; rd(R_CRXDATA, v); crx.push_back(v[8:0]); end
      end
    end
  endtask
endclass : i2c_ip_ctrl_cmd_vseq

// ---------------------------------------------------------------- external controller (VIP master)
class i2c_ip_ext_xfer_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_ext_xfer_vseq)

  rand i2c_ip_ext_xfer_e kind;
  rand i2c_ip_ext_dst_e  dst;
  rand bit [6:0]         other_addr;   // used with DST_OTHER
  rand bit [7:0]         data[];       // EX_WRITE payload
  rand int unsigned      rd_len = 1;   // EX_READ / EX_WRITE_READ
  rand bit [7:0]         ptr;          // EX_WRITE_READ pointer byte
  bit                    stop    = 1;  // EX_WRITE: 0 = keep the bus (next message starts with Sr)
  bit                    addr10  = 0;  // 10-bit header 1111_0xx (not supported by the IP, spec 1.4)
  int                    exp_ack = -1; // 1 / 0: complete acknowledge expected / refusal expected, -1: not checked
  bit                    ok;
  bit [6:0]              addr;
  bit [7:0]              rdata[$];

  // the VIP master sends at most 16 data bytes per message
  constraint c_len { data.size() <= 16; soft data.size() inside {[1:16]}; rd_len inside {[1:16]}; }
  constraint c_def { soft dst inside {DST_OWN0, DST_OWN1}; soft kind == EX_WRITE; }

  function new(string name = "i2c_ip_ext_xfer_vseq"); super.new(name); endfunction

  task main();
    bit [7:0] d[$];
    addr = (dst == DST_OWN0) ? own0 : (dst == DST_OWN1) ? own1 : other_addr;
    foreach (data[i]) d.push_back(data[i]);
    p_sequencer.ext.addr10 = addr10;
    case (kind)
      EX_WRITE:      ext_write(addr, d, ok, stop);
      EX_READ:       ext_read(addr, rd_len, rdata, ok);
      EX_WRITE_READ: ext_write_read(addr, ptr, rd_len, rdata, ok);
    endcase
    p_sequencer.ext.addr10 = 0;
    if (exp_ack >= 0)
      chk(ok == exp_ack, $sformatf("external %s 0x%02h: acknowledge %0d, expected %0d", kind.name(), addr, ok, exp_ack));
  endtask
endclass : i2c_ip_ext_xfer_vseq

// ---------------------------------------------------------------- background TRX reader
//   The target stretches when TRX lacks room, so long transfers need a concurrent drain. The level comes from
//   FIFO_STAT (debug_trx_lvl_o lags one cycle). Runs until stop_req and TRX empty.
class i2c_ip_trx_collect_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_trx_collect_vseq)

  bit       stop_req;
  realtime  poll = 3us;
  bit [9:0] q[$];

  function new(string name = "i2c_ip_trx_collect_vseq"); super.new(name); endfunction

  task main();
    forever begin
      int unsigned lvl;
      rd_field(R_FIFO_STAT, 24, 6, lvl);
      if (lvl == 0 && stop_req) break;
      if (lvl == 0) #(poll);
      repeat (lvl) begin bit [31:0] v; rd(R_TRXDATA, v); q.push_back(v[9:0]); end
    end
  endtask
endclass : i2c_ip_trx_collect_vseq

// ---------------------------------------------------------------- TX_REQ service (6.11)
//   On TX_REQ: W1C, drain TRX (into rx_log), load the next bytes of src into TTX. Runs until stop_req.
class i2c_ip_ttx_service_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_ttx_service_vseq)

  bit       stop_req;
  bit [7:0] src[$];
  bit [9:0] rx_log[$];

  function new(string name = "i2c_ip_ttx_service_vseq"); super.new(name); endfunction

  task main();
    while (!stop_req) begin
      bit [9:0] q[$];
      int unsigned lvl, room;
      fork begin
        fork
          wait (side.d_intr_stat[I_TX_REQ] === 1'b1);
          wait (stop_req);
        join_any
        disable fork;
      end join
      if (stop_req) break;
      w1c(32'h1 << I_TX_REQ);
      drain_trx(q);
      foreach (q[i]) rx_log.push_back(q[i]);
      rd_field(R_FIFO_STAT, 16, 6, lvl);
      room = depth - lvl;
      while (room != 0 && src.size() != 0) begin wr(R_TTXDATA, src.pop_front()); room--; end
    end
  endtask
endclass : i2c_ip_ttx_service_vseq

// ---------------------------------------------------------------- DMA operation start (6.15)
class i2c_ip_dma_start_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_dma_start_vseq)

  rand dma_sel_e    sel;
  rand bit          pfc;
  rand int unsigned count = 1;      // DMAC flow control: elements; PFC: buffer size
  rand int unsigned blk;            // DMA_BLK.BLK (and DMAC block size)
  bit               deny_single;    // DMAC DENYs SINGLE while >= blk elements remain
  bit               okay_as_rsvd;   // DMAC answers OKAY with the reserved code 2'b11
  bit               prog_blk = 1;   // write DMA_BLK
  bit [31:0]        tx_data[$];     // CMD / TTX words
  i2c_dma_op        op;

  constraint c_def { count inside {[1:64]}; blk <= 63; }

  function new(string name = "i2c_ip_dma_start_vseq"); super.new(name); endfunction

  task main();
    op = mk_op(sel, pfc, count, blk, deny_single);
    op.okay_as_rsvd = okay_as_rsvd;
    op.tx_data = tx_data;
    p_sequencer.dmac.start_op(op);
    if (prog_blk) wr(R_DMA_BLK, blk);
    wr(R_DMA_CTRL, {26'h0, sel, 2'b00, pfc, 1'b1});
  endtask
endclass : i2c_ip_dma_start_vseq

class i2c_ip_dma_check_vseq extends i2c_ip_base_vseq;
  `uvm_object_utils(i2c_ip_dma_check_vseq)

  i2c_dma_op op;
  string     exp_log  = "";         // expected request sequence ("" = not checked)
  bit        done_chk = 1;          // DMA_DONE / DMA_STAT / DMA_CTRL.EN checks
  realtime   tmo      = 5ms;
  string     what     = "DMA";

  function new(string name = "i2c_ip_dma_check_vseq"); super.new(name); endfunction

  task main();
    wait_op(op, tmo, what);
    if (exp_log != "") chk_dma_log(op, exp_log, what);
    if (done_chk) chk_dma_done(what);
  endtask
endclass : i2c_ip_dma_check_vseq
