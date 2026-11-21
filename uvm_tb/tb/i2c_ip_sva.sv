// -----------------------------------------------------------------------------
// i2c_ip_sva - port-level (black-box) assertions of apb_i2c, derived from the specification
//
//   ID               spec      rule
//   SVA_PREADY       3.1/3.6   PREADY is always 1 (zero wait state)
//   SVA_RST_OUT      6.1.1     outputs at their reset value while PRESETn = 0
//   SVA_CS_OE        3.2/R3    scl_cs_en_o and scl_oe_o never 1 together
//   SVA_CS_HS        R3        scl_cs_en_o only in Hs-mode (hs_mode_o = 1)
//   SVA_IRQ*         4.8.1     irq_* = registered OR of (INTR_STAT & INTR_EN) per group; debug_intr_stat_o is
//                              INTR_STAT delayed one cycle, so irq_x(t) = |(debug_intr_stat_o(t) & INTR_EN(t-1))
//   SVA_DMA_HOLD     4.9.3/R8  a request is never withdrawn before ack is sampled
//   SVA_DMA_TYPE     3.4/R8    req_type stable while req = 1
//   SVA_DMA_DROP     4.9.3     req returns to 0 the cycle after ack = 1 is sampled
//   SVA_DMA_NEXT     4.9.3/R8  a new request only after ack was sampled 0 (earliest one cycle after)
//   SVA_EN0_REL      4.1/4.10  CTRL.EN = 0: SCL/SDA released unless BUSMON override; override drives ~xxx_OVRD
//   SVA_EN0_HS       4.3.4     CTRL.EN = 0: hs_mode_o = 0, scl_cs_en_o = 0
//   SVA_NO_X         7.1       outputs never unknown after reset
//
//   INTR_EN / CTRL.EN / BUSMON are mirrored from the APB pins (write takes effect at the end of the access phase,
//   spec 3.6; SWRST clears CTRL.EN and keeps INTR_EN / BUSMON[10:8], spec 5.2.1).
// -----------------------------------------------------------------------------
`ifndef I2C_IP_SVA_SV
`define I2C_IP_SVA_SV

`include "uvm_macros.svh"

module i2c_ip_sva (
  input logic        PCLK,
  input logic        PRESETn,
  input logic        PSEL,
  input logic        PENABLE,
  input logic        PWRITE,
  input logic [11:0] PADDR,
  input logic [31:0] PWDATA,
  input logic [3:0]  PSTRB,
  input logic        PREADY,
  input logic        PSLVERR,
  input logic [31:0] PRDATA,
  input logic        scl_oe_o,
  input logic        sda_oe_o,
  input logic        scl_cs_en_o,
  input logic        hs_mode_o,
  input logic        irq_o,
  input logic        irq_ctrl_o,
  input logic        irq_tgt_o,
  input logic        irq_err_o,
  input logic        dma_req_o,
  input logic [1:0]  dma_req_type_o,
  input logic        dma_ack_i,
  input logic [1:0]  dma_ack_type_i,
  input logic [21:0] debug_intr_stat_o,
  input logic        debug_scl_o,
  input logic        debug_sda_o,
  input logic        debug_bus_busy_o,
  input logic        debug_ctrl_active_o,
  input logic        debug_tgt_hold_o,
  input logic [5:0]  debug_cmd_lvl_o
);
  import uvm_pkg::*;

  bit sva_en = 1'b1;               // tests may switch the checks off around deliberate protocol abuse

  // ---------------------------------------------------------------- APB register mirrors
  wire        apb_wr = PSEL & PENABLE & PWRITE;
  wire [9:0]  waddr  = PADDR[11:2];
  logic [21:0] en_sh;
  logic        ctrl_en_sh;
  logic [2:0]  busmon_sh;          // {SDA_OVRD, SCL_OVRD, OVRD_EN}
  int unsigned en0_stable;         // cycles since CTRL.EN / BUSMON mirror last changed

  always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
      en_sh      <= '0;
      ctrl_en_sh <= 1'b0;
      busmon_sh  <= 3'b110;
    end else if (apb_wr) begin
      if (waddr == 10'h003) begin
        if (PSTRB[0]) en_sh[7:0]   <= PWDATA[7:0];
        if (PSTRB[1]) en_sh[15:8]  <= PWDATA[15:8];
        if (PSTRB[2]) en_sh[21:16] <= PWDATA[21:16];
      end
      if (waddr == 10'h000) begin
        if (PSTRB[3] && PWDATA[31]) ctrl_en_sh <= 1'b0;
        else if (PSTRB[0])          ctrl_en_sh <= PWDATA[0];
      end
      if (waddr == 10'h01C && PSTRB[1]) busmon_sh <= PWDATA[10:8];
    end
  end

  logic ctrl_en_q;
  logic [2:0] busmon_q;
  always_ff @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) begin
      en0_stable <= 0; ctrl_en_q <= 1'b0; busmon_q <= 3'b110;
    end else begin
      ctrl_en_q <= ctrl_en_sh; busmon_q <= busmon_sh;
      en0_stable <= ((ctrl_en_q != ctrl_en_sh) || (busmon_q != busmon_sh)) ? 0 : ((en0_stable < 1000) ? en0_stable + 1 : en0_stable);
    end
  end

  function automatic void fail(string id, string msg);
    `uvm_error("I2C_SVA", $sformatf("[%s] %s", id, msg))
  endfunction

  default clocking cb @(posedge PCLK); endclocking

  // ---------------------------------------------------------------- APB
  A_PREADY: assert property (@(posedge PCLK) disable iff (!sva_en) PREADY === 1'b1)
    else fail("SVA_PREADY", "PREADY must always be 1");

  // ---------------------------------------------------------------- reset values (6.1.1)
  A_RST_OUT: assert property (@(posedge PCLK) disable iff (!sva_en)
      !PRESETn |-> (scl_oe_o == 1'b0 && sda_oe_o == 1'b0 && scl_cs_en_o == 1'b0 && hs_mode_o == 1'b0 &&
                    irq_o == 1'b0 && irq_ctrl_o == 1'b0 && irq_tgt_o == 1'b0 && irq_err_o == 1'b0 &&
                    dma_req_o == 1'b0 && dma_req_type_o == 2'b00 && PSLVERR == 1'b0 &&
                    debug_intr_stat_o == '0 && debug_scl_o == 1'b1 && debug_sda_o == 1'b1 &&
                    debug_bus_busy_o == 1'b0 && debug_ctrl_active_o == 1'b0 && debug_cmd_lvl_o == '0))
    else fail("SVA_RST_OUT", "output not at its reset value during PRESETn = 0");

  // ---------------------------------------------------------------- pads / Hs (3.2, R3)
  A_CS_OE: assert property (disable iff (!PRESETn || !sva_en) !(scl_cs_en_o && scl_oe_o))
    else fail("SVA_CS_OE", "scl_cs_en_o and scl_oe_o active together");

  A_CS_HS: assert property (disable iff (!PRESETn || !sva_en) scl_cs_en_o |-> hs_mode_o)
    else fail("SVA_CS_HS", "scl_cs_en_o active outside Hs-mode");

  // ---------------------------------------------------------------- interrupts (4.8.1)
  A_IRQ: assert property (disable iff (!PRESETn || !sva_en)
      ##1 (irq_o == |(debug_intr_stat_o & $past(en_sh))))
    else fail("SVA_IRQ", $sformatf("irq_o=%b, INTR_STAT=%h INTR_EN=%h", irq_o, debug_intr_stat_o, $past(en_sh)));

  A_IRQ_CTRL: assert property (disable iff (!PRESETn || !sva_en)
      ##1 (irq_ctrl_o == |(debug_intr_stat_o[7:0] & $past(en_sh[7:0]))))
    else fail("SVA_IRQ_CTRL", "irq_ctrl_o is not the OR of INTR_STAT[7:0] & INTR_EN[7:0]");

  A_IRQ_TGT: assert property (disable iff (!PRESETn || !sva_en)
      ##1 (irq_tgt_o == |(debug_intr_stat_o[15:8] & $past(en_sh[15:8]))))
    else fail("SVA_IRQ_TGT", "irq_tgt_o is not the OR of INTR_STAT[15:8] & INTR_EN[15:8]");

  A_IRQ_ERR: assert property (disable iff (!PRESETn || !sva_en)
      ##1 (irq_err_o == |(debug_intr_stat_o[20:16] & $past(en_sh[20:16]))))
    else fail("SVA_IRQ_ERR", "irq_err_o is not the OR of INTR_STAT[20:16] & INTR_EN[20:16]");

  // ---------------------------------------------------------------- DMA handshake (4.9.3, R8)
  A_DMA_HOLD: assert property (disable iff (!PRESETn || !sva_en) (dma_req_o && !dma_ack_i) |=> dma_req_o)
    else fail("SVA_DMA_HOLD", "dma_req_o withdrawn before dma_ack_i");

  A_DMA_TYPE: assert property (disable iff (!PRESETn || !sva_en) (dma_req_o && $past(dma_req_o)) |-> $stable(dma_req_type_o))
    else fail("SVA_DMA_TYPE", "dma_req_type_o changed while dma_req_o = 1");

  A_DMA_DROP: assert property (disable iff (!PRESETn || !sva_en) (dma_req_o && dma_ack_i) |=> !dma_req_o)
    else fail("SVA_DMA_DROP", "dma_req_o not cleared the cycle after dma_ack_i = 1 was sampled");

  A_DMA_NEXT: assert property (disable iff (!PRESETn || !sva_en) $rose(dma_req_o) |-> ($past(!dma_ack_i, 1) && $past(!dma_ack_i, 2)))
    else fail("SVA_DMA_NEXT", "new dma_req_o before dma_ack_i was sampled low");

  // ---------------------------------------------------------------- CTRL.EN = 0 (4.1, 4.3.4, 4.10)
  A_EN0_REL: assert property (disable iff (!PRESETn || !sva_en)
      (!ctrl_en_sh && en0_stable >= 3 && $stable(ctrl_en_sh) && $stable(busmon_sh)) |-> (scl_oe_o == (busmon_sh[0] & ~busmon_sh[1]) && sda_oe_o == (busmon_sh[0] & ~busmon_sh[2])))
    else fail("SVA_EN0_REL", $sformatf("CTRL.EN=0: scl_oe_o=%b sda_oe_o=%b with BUSMON[10:8]=%b", scl_oe_o, sda_oe_o, busmon_sh));

  A_EN0_HS: assert property (disable iff (!PRESETn || !sva_en)
      (!ctrl_en_sh && en0_stable >= 3 && $stable(ctrl_en_sh) && $stable(busmon_sh)) |-> (!hs_mode_o && !scl_cs_en_o))
    else fail("SVA_EN0_HS", "CTRL.EN=0 but hs_mode_o / scl_cs_en_o active");

  // ---------------------------------------------------------------- X checks (7.1)
  A_NO_X: assert property (disable iff (!PRESETn || !sva_en)
      !$isunknown({scl_oe_o, sda_oe_o, scl_cs_en_o, hs_mode_o, irq_o, irq_ctrl_o, irq_tgt_o, irq_err_o, dma_req_o, dma_req_type_o, PREADY, PSLVERR}))
    else fail("SVA_NO_X", "unknown value on an output");

  A_NO_X_RD: assert property (disable iff (!PRESETn || !sva_en) (PSEL && PENABLE && !PWRITE) |-> !$isunknown(PRDATA))
    else fail("SVA_NO_X", "PRDATA unknown during a read");

  // ---------------------------------------------------------------- functional cover
  C_CS_EN:   cover property (disable iff (!PRESETn) $rose(scl_cs_en_o));
  C_HS:      cover property (disable iff (!PRESETn) $rose(hs_mode_o));
  C_DMA_REQ: cover property (disable iff (!PRESETn) dma_req_o && dma_ack_i);
  C_IRQ_ERR: cover property (disable iff (!PRESETn) $rose(irq_err_o));
endmodule : i2c_ip_sva

`endif
