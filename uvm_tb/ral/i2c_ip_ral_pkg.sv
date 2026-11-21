// -----------------------------------------------------------------------------
// i2c_ip_ral_pkg - UVM register model of apb_i2c (spec 5.1 / 5.2)
//
//   One generic register class (fields are added by the block from the spec table); every register handle is a named
//   member of i2c_ip_reg_block. Access policies:
//     RW, RO, WO, W1C; W1SC bits are modelled as "W1S" + volatile (HW self-clear); HW-updated fields are volatile.
//   Attributes: FIFO data registers (pop/push side effects) and W1SC / write-sensitive registers carry
//   NO_REG_TESTS so that generic register sequences (uvm_reg_hw_reset_seq, uvm_reg_bit_bash_seq) never touch them;
//   FIFO_CTRL carries NO_REG_BIT_BASH_TEST (clamped thresholds).
//   i2c_ip_reg_adapter extends the APB VIP adapter to report the write strobes to the predictor (byte enables).
// -----------------------------------------------------------------------------
`ifndef I2C_IP_RAL_PKG_SV
`define I2C_IP_RAL_PKG_SV

`include "uvm_macros.svh"

package i2c_ip_ral_pkg;
  import uvm_pkg::*;
  import apb_pkg::*;

  // ---------------------------------------------------------------- generic register
  class i2c_ip_reg extends uvm_reg;
    `uvm_object_utils(i2c_ip_reg)
    function new(string name = "i2c_ip_reg");
      super.new(name, 32, UVM_NO_COVERAGE);
    endfunction
    function uvm_reg_field add(string fname, int unsigned lsb, int unsigned size, string access,
                               uvm_reg_data_t rst, bit vol = 0);
      uvm_reg_field f = uvm_reg_field::type_id::create(fname);
      f.configure(this, size, lsb, access, vol, rst, 1, 1, 0);
      return f;
    endfunction
    // every field is plain RW (a partial-strobe write can be predicted by merging with the mirror)
    function bit all_rw();
      uvm_reg_field fl[$];
      get_fields(fl);
      foreach (fl[i]) if (fl[i].get_access() != "RW") return 0;
      return fl.size() != 0;
    endfunction
    // mask of the defined (non-reserved) bits
    function uvm_reg_data_t defined_mask();
      uvm_reg_field fl[$];
      uvm_reg_data_t m = 0;
      get_fields(fl);
      foreach (fl[i]) m |= ((uvm_reg_data_t'(1) << fl[i].get_n_bits()) - 1) << fl[i].get_lsb_pos();
      return m;
    endfunction
  endclass : i2c_ip_reg

  // ---------------------------------------------------------------- block
  class i2c_ip_reg_block extends uvm_reg_block;
    `uvm_object_utils(i2c_ip_reg_block)

    int unsigned fifo_depth = 16;
    bit [23:0]   devid_rst  = 24'h0;
    bit [31:0]   version    = 32'h0002_0000;

    rand i2c_ip_reg CTRL, STATUS, INTR_STAT, INTR_EN, INTR_TEST, FIFO_CTRL, FIFO_STAT, CMD, CRXDATA, TTXDATA, TRXDATA;
    rand i2c_ip_reg TADDR, TCFG, HSCFG, CSTAT, TSTAT;
    rand i2c_ip_reg TIM_SCL, TIM_STA, TIM_STO, TIM_DAT, HS_TIM_SCL, HS_TIM_STA, HS_TIM_STO, HS_TIM_DAT;
    rand i2c_ip_reg FILTER, TIMEOUT, BUSIDLE, DEVID, BUSMON, DMA_CTRL, DMA_BLK, DMA_STAT, PARAM, VERSION;

    i2c_ip_reg all_regs[$];
    i2c_ip_reg timing_regs[$];              // "RW (lock)": writes ignored while CTRL.EN = 1

    function new(string name = "i2c_ip_reg_block");
      super.new(name, UVM_NO_COVERAGE);
    endfunction

    protected function i2c_ip_reg mk(string n, uvm_reg_addr_t off, bit lock = 0, bit no_tests = 0);
      i2c_ip_reg r = i2c_ip_reg::type_id::create(n);
      r.configure(this, null, "");
      default_map.add_reg(r, off, (n == "INTR_TEST" || n == "CMD" || n == "TTXDATA") ? "WO" :
                                  (n == "STATUS" || n == "FIFO_STAT" || n == "CRXDATA" || n == "TRXDATA" ||
                                   n == "TSTAT" || n == "PARAM" || n == "VERSION") ? "RO" : "RW");
      if (no_tests) uvm_resource_db #(bit)::set({"REG::", r.get_full_name()}, "NO_REG_TESTS", 1, this);
      all_regs.push_back(r);
      if (lock) timing_regs.push_back(r);
      return r;
    endfunction

    virtual function void build();
      uvm_reg_field f;
      default_map = create_map("apb_map", 0, 4, UVM_LITTLE_ENDIAN, 1);

      CTRL = mk("CTRL", 'h000, 0, 1);
      f = CTRL.add("EN", 0, 1, "RW", 0);         f = CTRL.add("CTRL_EN", 1, 1, "RW", 0);
      f = CTRL.add("TGT_EN", 2, 1, "RW", 0);     f = CTRL.add("HS_EN", 3, 1, "RW", 0);
      f = CTRL.add("STARTBYTE_EN", 4, 1, "RW", 0);
      f = CTRL.add("CMD_FLUSH", 8, 1, "W1S", 0, 1);  f = CTRL.add("CRX_FLUSH", 9, 1, "W1S", 0, 1);
      f = CTRL.add("TTX_FLUSH", 10, 1, "W1S", 0, 1); f = CTRL.add("TRX_FLUSH", 11, 1, "W1S", 0, 1);
      f = CTRL.add("ABORT", 16, 1, "W1S", 0, 1);     f = CTRL.add("BUSCLR", 17, 1, "W1S", 0, 1);
      f = CTRL.add("SWRST", 31, 1, "W1S", 0, 1);

      STATUS = mk("STATUS", 'h004);
      f = STATUS.add("BUS_BUSY", 0, 1, "RO", 0, 1);    f = STATUS.add("CTRL_ACTIVE", 1, 1, "RO", 0, 1);
      f = STATUS.add("CTRL_HOLD", 2, 1, "RO", 0, 1);   f = STATUS.add("TGT_ACTIVE", 3, 1, "RO", 0, 1);
      f = STATUS.add("TGT_RW", 4, 1, "RO", 0, 1);      f = STATUS.add("TGT_HOLD", 5, 1, "RO", 0, 1);
      f = STATUS.add("HS_ACTIVE", 6, 1, "RO", 0, 1);   f = STATUS.add("BUSCLR_ACTIVE", 7, 1, "RO", 0, 1);
      f = STATUS.add("CMD_FULL", 8, 1, "RO", 0, 1);    f = STATUS.add("CMD_EMPTY", 9, 1, "RO", 1, 1);
      f = STATUS.add("CRX_FULL", 10, 1, "RO", 0, 1);   f = STATUS.add("CRX_EMPTY", 11, 1, "RO", 1, 1);
      f = STATUS.add("TTX_FULL", 12, 1, "RO", 0, 1);   f = STATUS.add("TTX_EMPTY", 13, 1, "RO", 1, 1);
      f = STATUS.add("TRX_FULL", 14, 1, "RO", 0, 1);   f = STATUS.add("TRX_EMPTY", 15, 1, "RO", 1, 1);

      INTR_STAT = mk("INTR_STAT", 'h008, 0, 1);
      f = INTR_STAT.add("CMD_THR", 0, 1, "RO", 1, 1);   f = INTR_STAT.add("CRX_THR", 1, 1, "RO", 0, 1);
      f = INTR_STAT.add("CTRL_EV", 2, 6, "W1C", 0, 1);
      f = INTR_STAT.add("TRX_THR", 8, 1, "RO", 0, 1);   f = INTR_STAT.add("TTX_THR", 9, 1, "RO", 1, 1);
      f = INTR_STAT.add("TGT_EV", 10, 6, "W1C", 0, 1);
      f = INTR_STAT.add("ERR_EV", 16, 5, "W1C", 0, 1);  f = INTR_STAT.add("DMA_DONE", 21, 1, "W1C", 0, 1);

      INTR_EN   = mk("INTR_EN", 'h00C);   f = INTR_EN.add("EN", 0, 22, "RW", 0);
      INTR_TEST = mk("INTR_TEST", 'h010, 0, 1); f = INTR_TEST.add("TEST", 0, 22, "WO", 0);

      FIFO_CTRL = mk("FIFO_CTRL", 'h014);
      // thresholds >= FIFO_DEPTH are clamped (5.2.4): the read-back of a clamped value is not defined by the spec
      uvm_resource_db #(bit)::set({"REG::", FIFO_CTRL.get_full_name()}, "NO_REG_BIT_BASH_TEST", 1, this);
      f = FIFO_CTRL.add("CMD_THR", 0, 6, "RW", 0);  f = FIFO_CTRL.add("CRX_THR", 8, 6, "RW", 0);
      f = FIFO_CTRL.add("TTX_THR", 16, 6, "RW", 0); f = FIFO_CTRL.add("TRX_THR", 24, 6, "RW", 0);

      FIFO_STAT = mk("FIFO_STAT", 'h018);
      f = FIFO_STAT.add("CMD_LVL", 0, 6, "RO", 0, 1);  f = FIFO_STAT.add("CRX_LVL", 8, 6, "RO", 0, 1);
      f = FIFO_STAT.add("TTX_LVL", 16, 6, "RO", 0, 1); f = FIFO_STAT.add("TRX_LVL", 24, 6, "RO", 0, 1);

      CMD     = mk("CMD", 'h01C, 0, 1);     f = CMD.add("CMD", 0, 13, "WO", 0);
      CRXDATA = mk("CRXDATA", 'h020, 0, 1); f = CRXDATA.add("DATA", 0, 8, "RO", 0, 1); f = CRXDATA.add("LAST", 8, 1, "RO", 0, 1);
      TTXDATA = mk("TTXDATA", 'h024, 0, 1); f = TTXDATA.add("DATA", 0, 8, "WO", 0);
      TRXDATA = mk("TRXDATA", 'h028, 0, 1); f = TRXDATA.add("DATA", 0, 8, "RO", 0, 1); f = TRXDATA.add("TAG", 8, 2, "RO", 0, 1);

      TADDR = mk("TADDR", 'h02C);
      f = TADDR.add("ADDR0", 0, 7, "RW", 0); f = TADDR.add("ADDR0_EN", 7, 1, "RW", 0);
      f = TADDR.add("ADDR1", 8, 7, "RW", 0); f = TADDR.add("ADDR1_EN", 15, 1, "RW", 0);
      TCFG = mk("TCFG", 'h030);
      f = TCFG.add("GC_EN", 0, 1, "RW", 0);  f = TCFG.add("NOSTRETCH", 1, 1, "RW", 0);
      f = TCFG.add("TNACK", 2, 1, "RW", 0);  f = TCFG.add("DEVID_EN", 3, 1, "RW", 0);
      HSCFG = mk("HSCFG", 'h034); f = HSCFG.add("MCODE", 0, 3, "RW", 1);
      CSTAT = mk("CSTAT", 'h038, 0, 1); f = CSTAT.add("CSTAT", 0, 7, "W1C", 0, 1);
      TSTAT = mk("TSTAT", 'h03C);
      f = TSTAT.add("MATCH_ADDR", 0, 7, "RO", 0, 1); f = TSTAT.add("RW", 7, 1, "RO", 0, 1);
      f = TSTAT.add("GC", 8, 1, "RO", 0, 1);         f = TSTAT.add("ADDR1_HIT", 9, 1, "RO", 0, 1);
      f = TSTAT.add("DEVID_ACTIVE", 10, 1, "RO", 0, 1);

      TIM_SCL = mk("TIM_SCL", 'h040, 1); f = TIM_SCL.add("SCL_LOW", 0, 16, "RW", 500); f = TIM_SCL.add("SCL_HIGH", 16, 16, "RW", 490);
      TIM_STA = mk("TIM_STA", 'h044, 1); f = TIM_STA.add("THD_STA", 0, 16, "RW", 400); f = TIM_STA.add("TSU_STA", 16, 16, "RW", 470);
      TIM_STO = mk("TIM_STO", 'h048, 1); f = TIM_STO.add("TSU_STO", 0, 16, "RW", 400); f = TIM_STO.add("TBUF", 16, 16, "RW", 470);
      TIM_DAT = mk("TIM_DAT", 'h04C, 1); f = TIM_DAT.add("THD_DAT", 0, 16, "RW", 30);  f = TIM_DAT.add("TSU_DAT", 16, 16, "RW", 125);
      HS_TIM_SCL = mk("HS_TIM_SCL", 'h050, 1); f = HS_TIM_SCL.add("SCL_LOW", 0, 16, "RW", 15); f = HS_TIM_SCL.add("SCL_HIGH", 16, 16, "RW", 5);
      HS_TIM_STA = mk("HS_TIM_STA", 'h054, 1); f = HS_TIM_STA.add("THD_STA", 0, 16, "RW", 16); f = HS_TIM_STA.add("TSU_STA", 16, 16, "RW", 16);
      HS_TIM_STO = mk("HS_TIM_STO", 'h058, 1); f = HS_TIM_STO.add("TSU_STO", 0, 16, "RW", 16);
      HS_TIM_DAT = mk("HS_TIM_DAT", 'h05C, 1); f = HS_TIM_DAT.add("THD_DAT", 0, 16, "RW", 1);  f = HS_TIM_DAT.add("TSU_DAT", 16, 16, "RW", 9);
      FILTER  = mk("FILTER", 'h060, 1);  f = FILTER.add("SPKLEN_FS", 0, 8, "RW", 5); f = FILTER.add("SPKLEN_HS", 8, 8, "RW", 1);
      TIMEOUT = mk("TIMEOUT", 'h064);    f = TIMEOUT.add("TOUT_VAL", 0, 24, "RW", 'h3567E0); f = TIMEOUT.add("TOUT_EN", 31, 1, "RW", 0);
      BUSIDLE = mk("BUSIDLE", 'h068, 1); f = BUSIDLE.add("IDLE_VAL", 0, 16, "RW", 1000);
      DEVID   = mk("DEVID", 'h06C);      f = DEVID.add("DEVID", 0, 24, "RW", devid_rst);
      BUSMON  = mk("BUSMON", 'h070, 0, 1);
      f = BUSMON.add("SCL_F", 0, 1, "RO", 1, 1);   f = BUSMON.add("SDA_F", 1, 1, "RO", 1, 1);
      f = BUSMON.add("SCL_OE", 2, 1, "RO", 0, 1);  f = BUSMON.add("SDA_OE", 3, 1, "RO", 0, 1);
      f = BUSMON.add("OVRD_EN", 8, 1, "RW", 0);    f = BUSMON.add("SCL_OVRD", 9, 1, "RW", 1);
      f = BUSMON.add("SDA_OVRD", 10, 1, "RW", 1);
      DMA_CTRL = mk("DMA_CTRL", 'h074, 0, 1);
      f = DMA_CTRL.add("EN", 0, 1, "RW", 0, 1); f = DMA_CTRL.add("PFC", 1, 1, "RW", 0); f = DMA_CTRL.add("SEL", 4, 2, "RW", 0);
      DMA_BLK  = mk("DMA_BLK", 'h078, 0, 1);  f = DMA_BLK.add("BLK", 0, 6, "RW", 0);
      DMA_STAT = mk("DMA_STAT", 'h07C, 0, 1);
      f = DMA_STAT.add("REQ", 0, 1, "RO", 0, 1); f = DMA_STAT.add("DONE", 8, 1, "W1C", 0, 1); f = DMA_STAT.add("DENY", 16, 1, "W1C", 0, 1);
      PARAM   = mk("PARAM", 'h0F8);
      f = PARAM.add("FIFO_DEPTH", 0, 8, "RO", fifo_depth); f = PARAM.add("HS_SUP", 8, 1, "RO", 1);
      f = PARAM.add("DEVID_SUP", 9, 1, "RO", 1);           f = PARAM.add("DMA_SUP", 10, 1, "RO", 1);
      VERSION = mk("VERSION", 'h0FC);    f = VERSION.add("VERSION", 0, 32, "RO", version);

      lock_model();
    endfunction

    function i2c_ip_reg by_offset(uvm_reg_addr_t off);
      foreach (all_regs[i]) if (all_regs[i].get_offset() == off) return all_regs[i];
      return null;
    endfunction
  endclass : i2c_ip_reg_block

  // ---------------------------------------------------------------- adapter with byte enables
  //   uvm_reg_field::do_predict only looks at the byte enable of the lowest byte of a field, so a partial-strobe
  //   write to a field spanning several byte lanes would be mispredicted. For registers made only of RW fields the
  //   adapter merges the strobed lanes with the mirrored value and reports a full write (spec 3.6: only the
  //   strobed lanes change).
  class i2c_ip_reg_adapter extends apb_reg_adapter;
    `uvm_object_utils(i2c_ip_reg_adapter)
    i2c_ip_reg_block ral;                       // set by the env
    function new(string name = "i2c_ip_reg_adapter"); super.new(name); endfunction
    virtual function void bus2reg(uvm_sequence_item bus_item, ref uvm_reg_bus_op rw);
      apb_item t;
      super.bus2reg(bus_item, rw);
      if ($cast(t, bus_item)) begin
        rw.byte_en = t.write ? uvm_reg_byte_en_t'(t.strb) : '1;
        rw.addr    = t.addr & 'hFFC;              // PADDR[1:0] ignored (spec 3.1)
        if (t.write && ral != null && t.strb != 4'hF && t.strb != 4'h0) begin
          i2c_ip_reg r = ral.by_offset(rw.addr);
          if (r != null && r.all_rw()) begin
            uvm_reg_data_t m = 0;
            for (int k = 0; k < 4; k++) if (t.strb[k]) m[8*k +: 8] = 8'hFF;
            rw.data    = (r.get_mirrored_value() & ~m) | (rw.data & m);
            rw.byte_en = '1;
          end
        end
      end
    endfunction
  endclass : i2c_ip_reg_adapter

endpackage : i2c_ip_ral_pkg

`endif
