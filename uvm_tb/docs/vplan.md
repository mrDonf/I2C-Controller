# Verification Plan — apb_i2c

Tài liệu này là Verification Plan (VP) của IP `apb_i2c`, viết từ đặc tả `specs/i2c_architecture_spec.md` (dựa trên
UM10204 của NXP, trừ các tính năng ở mục 1.4). Mỗi **VP item** là một testcase UVM riêng: một class kế thừa
`i2c_ip_base_test` (qua base test của nhóm), một file trong `tests/<nhóm>/`. Testcase tự gọi tổ hợp sequence của nó
trong `run_scenario()`: tuần tự, song song (`fork`), randomize kèm inline constraint, hoặc thay thế bằng factory override.

## 1. Phương pháp

| Thành phần | Vai trò |
|---|---|
| Kiểm tra theo spec (black-box) | Kỳ vọng chỉ suy ra từ đặc tả (bảng thanh ghi, timing 4.4.2, ví dụ DMA 4.9.4, quy trình mục 6). Từ RTL chỉ dùng danh sách port (mục 3). |
| Scoreboard / reference model | CMD → bus → CRX, bus → TRX (tag), TTX → bus, Device ID, bảng giải mã địa chỉ 4.5.1, luật APB (PSLVERR, đọc 0) — kiểm mọi message trong mọi test. |
| SVA (mức port) | R3, R8 (handshake request DMA, rút request bằng `DMA_CTRL.CLR`, `dma_end_ack_o` = `dma_end_req_i` trễ 1 chu kỳ), R9 (giao thức Q-Channel, `PREADY = QACCEPTn`, chỉ chấp nhận khi rảnh, reset theo `QREQN_RST`), timing IRQ 4.8.1, giá trị reset 6.1.1, nhả bus khi `EN = 0`, không có X trên output. Các SVA chạy trên PCLK đã gate của DUT. |
| LPI clock controller | Agent `agents/qch` (`i2c_qch_ctrl`): mô hình clock controller + ICG (`tb/i2c_ip_qch_if.sv`) — yêu cầu dừng, gate PCLK, tự đánh thức khi `QACTIVE` (độ trễ ngẫu nhiên), phân loại nguồn đánh thức, kiểm tra thời gian đáp ứng và "chỉ chấp nhận khi rảnh". APB VIP, DMAC và VIP I2C chạy trên clock không gate (như bridge / DMAC ngoài miền clock). |
| VIP monitor | Giao thức APB, giao thức I2C và timing UM10204 theo mode. |
| Self-check trong sequence | Event `INTR_STAT`, `CSTAT`, `STATUS`, `TSTAT`, mức FIFO, danh sách entry TRX, bộ nhớ VIP slave, đo timing bus. |
| Register model | `uvm_reg_hw_reset_seq`, `uvm_reg_bit_bash_seq`, mirror check (adapter dự đoán đúng ghi một phần byte lane). |
| Coverage | Covergroup chức năng (`i2c_ip_coverage`) + code coverage RTL (`+cover=sbcef`) + coverage assertion. |

## 2. Cấu trúc sequence (được tham chiếu ở bảng mục 4)

| Lớp | Thư mục | Ví dụ |
|---|---|---|
| L-1 agent | `VIP/`, `agents/` | APB VIP item, `i2c_api_seq`, `i2c_bb_api_seq`, `i2c_dma_op` |
| L0 API | `seq_lib/base` | `i2c_ip_base_vseq` (APB/RAL, `chk`, `wait_ev`, `push_cmds`, `ext_write`, `recover` ...) |
| L1 cấu hình | `seq_lib/cfg` | `i2c_ip_init_vseq`, `i2c_ip_hs_cfg_vseq`, `i2c_ip_timeout_cfg_vseq`, `i2c_ip_fifo_thr_vseq`, `i2c_ip_ext_dev_cfg_vseq` |
| L1 traffic | `seq_lib/xfer` | `i2c_ip_ctrl_xfer_vseq`, `i2c_ip_ctrl_cmd_vseq`, `i2c_ip_ext_xfer_vseq`, `i2c_ip_trx_collect_vseq`, `i2c_ip_ttx_service_vseq`, `i2c_ip_dma_start_vseq`, `i2c_ip_dma_check_vseq` |
| L1 bus | `seq_lib/bus` | `i2c_ip_bb_arb_vseq`, `i2c_ip_bb_scl_stretch_vseq`, `i2c_ip_bb_glitch_vseq`, `i2c_ip_bb_hold_vseq`, `i2c_ip_bb_start_pulse_vseq`, `i2c_ip_bb_stop_block_vseq` |
| L1 tiện ích | `seq_lib/util` | `i2c_ip_timing_meas_vseq`, `i2c_ip_recover_vseq`, `i2c_ip_presetn_vseq`, `i2c_ip_eot_vseq` |
| L1 low power | `seq_lib/lp` | `i2c_ip_qch_cycle_vseq` (một chu trình dừng / giữ / thoát), `i2c_ip_qch_bg_vseq` (LPI controller nền: yêu cầu ngẫu nhiên song song traffic) |
| L2 tính năng | `seq_lib/feature` | một vseq tự kiểm cho mỗi hạng mục (vd. `i2c_ctrl_abort_vseq`, `i2c_tgt_devid_vseq`) |
| Random | `seq_lib/rand` | `i2c_rand_traffic_vseq`, `i2c_rand_reg_vseq`, `i2c_ip_ctrl_xfer_stretch_vseq` (dùng cho override) |
| UVM chuẩn | `uvm_pkg` | `uvm_reg_hw_reset_seq`, `uvm_reg_bit_bash_seq` |

Mọi testcase đi qua các phase: `reset_phase` (chờ PRESETn) → `configure_phase` (`i2c_ip_init_vseq` randomize theo
knob của test, hook `randomize_init()`) → `main_phase` (`run_scenario()`) → `shutdown_phase` (`i2c_ip_eot_vseq`: bus
đã được nhả, không còn transfer, CMD/CRX/TRX rỗng).

## 3. Ma trận tính năng (spec 1.3) → VP item

| Tính năng | VP item |
|---|---|
| F1 APB4, PSTRB, PSLVERR | REG-01..10, FIFO-01..03 |
| F2 Controller / Target / cả hai | CTL-*, TGT-*, SYS-01, RND-01 |
| F3 Sm / Fm / Fm+ / Hs, timing lập trình | CTL-05 (3 test), CTL-16, RND-01 (+SPEED), RND-05 |
| F4 7-bit, ADDR0 / ADDR1 | TGT-01, TGT-03, TGT-15 |
| F5 S / Sr / P, ACK/NACK, độ dài không giới hạn | CTL-01..03, CTL-06/07, CTL-17/18, TGT-02, TGT-07 |
| F6 Clock sync, arbitration, sang target khi thua | BUS-01..04, BUS-13..17 |
| F7 Clock stretching | CTL-07, BUS-04, TGT-06, TGT-08, TGT-16, RND-04 |
| F8 General call, Software Reset | CTL-15, TGT-11 |
| F9 START byte | CTL-14, TGT-11 |
| F10 Bus clear | BUS-05; khôi phục trong BUS-06, BUS-07, BUS-11, BUS-12 |
| F11 Device ID | TGT-14, TGT-17, TGT-18, CTL-15 |
| F12 Hs-mode | CTL-16, TGT-12 (HS_DET) |
| F13 Reserved address | TGT-12, TGT-13 |
| F14 Synchronizer + spike filter | BUS-08 |
| F15 SCL timeout, bus error, bus idle | BUS-06, BUS-07, BUS-10, BUS-18 |
| F16 4 FIFO | FIFO-01..05, CTL-06/07, TGT-04, TGT-08/09 |
| F17 Ngắt | INT-01..03, CTL-04, CTL-08, FIFO-04/05, TGT-01 |
| F18 DMA | DMA-01..13 (DMA-12: handshake kết thúc; DMA-13: rút request / DMAC không có DENY); BUS-11 (SWRST với `DMA_STAT.END`, `END_EN`) |
| F19 Bus monitor / override | BUS-09, BUS-05 |
| F21 Q-Channel (dừng PCLK) | QCH-01..08; SVA R9 trong mọi test; QCH-07 + 5 test hiện có trên build `QREQN_RST = 0` |
| F20 Cổng trạng thái `debug_*` | quan sát trong mọi test (side monitor: `debug_intr_stat_o` → event ngắt, `debug_ctrl_active_o` → scoreboard, `debug_*_hold_o`); BUS-10 (`debug_bus_busy_o`); SVA (`irq_*` = `debug_intr_stat_o` & `INTR_EN`, giá trị reset) |
| 1.4 không hỗ trợ: 10-bit, CBUS, HW general call | TGT-13, TGT-12, TGT-11 (chỉ kiểm hành vi đã định nghĩa: NACK / dữ liệu thường) |

## 4. VP item → testcase → sequence

Cột *Regression*: `Nx dA,B` = N seed trên mỗi build FIFO_DEPTH A, B (`sim/regress.list`); `q0` = thêm build `QREQN_RST = 0`.

### Thanh ghi / APB (F1, 3.1, 3.6, 3.7, 5)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-REG-01 | `i2c_reg_hw_reset_test` | reset values: UVM uvm_reg_hw_reset_seq + explicit spec table + output ports after reset (5.1, 6.1.1) | `i2c_reset_outputs_vseq`, `uvm_reg_hw_reset_seq`, `i2c_reg_reset_val_vseq` | 1x d4,8,16,32 |
| VP-REG-02 | `i2c_reg_presetn_test` | PRESETn in the middle of activity: registers, FIFOs and outputs return to reset (6.1.1) | `i2c_ip_ctrl_xfer_vseq`, `i2c_reg_dirty_vseq`, `i2c_ip_presetn_vseq`, `uvm_reg_hw_reset_seq`, `i2c_reg_reset_val_vseq`, `i2c_reset_outputs_vseq` | 1x d4,16 |
| VP-REG-03 | `i2c_reg_bit_bash_test` | RW registers: UVM uvm_reg_bit_bash_seq + pattern / reserved-bit checks (5.1, 3.7) | `uvm_reg_bit_bash_seq`, `i2c_reg_rw_pattern_vseq` | 1x d4,32 |
| VP-REG-04 | `i2c_reg_strobe_test` | PSTRB byte lanes on RW, W1C and FIFO push registers (3.6) | `i2c_reg_strobe_vseq` | 1x d16 |
| VP-REG-05 | `i2c_reg_lock_test` | timing / FILTER / BUSIDLE locked while CTRL.EN = 1, the running configuration is not disturbed (3.7, 5.1) | `i2c_reg_lock_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d16 |
| VP-REG-06/07/08 | `i2c_reg_access_err_test` | RO / WO registers, unmapped offsets (PSLVERR), PADDR[1:0] and PPROT ignored (3.1, 3.7) | `i2c_reg_ro_wo_vseq`, `i2c_reg_unmapped_vseq`, `i2c_reg_paddr_pprot_vseq` | 2x d16 |
| VP-REG-09 | `i2c_reg_w1c_test` | W1C / W1SC registers, DMA_CTRL lane rule, register model mirror (5.2) | `i2c_reg_w1c_w1s_vseq` | 1x d16 |
| VP-REG-10 | `i2c_reg_rand_test` | random register accesses: values, strobes, PADDR[1:0], RO / unmapped offsets, mirror checks (+N_ITER) | `i2c_rand_reg_vseq` | 2x d4,32 |

### Ngắt (F17, 4.8)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-INT-01/03 | `i2c_intr_test` | event bits: INTR_TEST, W1C (lanes), INTR_EN routing to the 4 lines; level bits (4.8) | `i2c_intr_event_vseq`, `i2c_intr_level_vseq` | 1x d16 |
| VP-INT-02 | `i2c_intr_rand_test` | random event order, several events and random INTR_EN at once (4.8) | `i2c_intr_event_vseq`, `i2c_intr_multi_vseq` | 2x d16 |

### FIFO (F16, 4.7)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-FIFO-01 | `i2c_fifo_cmd_test` | CMD FIFO: level, threshold, full, overflow PSLVERR + FIFO_ERR, clamp, flush, kept with EN = 0 (4.7) | `i2c_fifo_cmd_vseq` | 1x d4,8,16,32 |
| VP-FIFO-02 | `i2c_fifo_ttx_test` | TTX FIFO: level, threshold, full, overflow PSLVERR + FIFO_ERR, flush (4.7) | `i2c_fifo_ttx_vseq` | 1x d4,8,16,32 |
| VP-FIFO-03/04/05 | `i2c_fifo_rx_test` | RX FIFOs: empty reads, CRX / TRX thresholds with real traffic and random levels, irq lines (3.7, 4.7, 4.8) | `i2c_fifo_rx_empty_vseq`, `i2c_fifo_crx_thr_vseq`, `i2c_fifo_trx_thr_vseq` | 1x d4,8,16,32 |

### Controller (F2, F3, F5, F8, F9, F11, F12; 4.4, 6.3 - 6.8)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-CTL-01 | `i2c_ctrl_write_test` | controller write (6.3): random targets / pointers / lengths, address-only write, idle state afterwards | `i2c_ip_ctrl_xfer_vseq`, `i2c_ctrl_idle_chk_vseq` | 1x d4,16,32 |
| VP-CTL-02 | `i2c_ctrl_read_test` | controller read (6.4): 1 byte .. longer than CRX, LAST on the final byte, both VIP targets | `i2c_ip_ctrl_xfer_vseq`, `i2c_ctrl_idle_chk_vseq` | 1x d4,16,32 |
| VP-CTL-03 | `i2c_ctrl_write_read_test` | register write then register read with repeated START (6.5): the read returns the written bytes | `i2c_ip_ctrl_xfer_vseq` | 1x d4,16,32 |
| VP-CTL-04 | `i2c_ctrl_irq_test` | interrupt-driven completion: irq_ctrl_o raised by CMPL, released by the W1C (6.12) | `i2c_ctrl_cmpl_irq_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d16 |
| VP-CTL-05 | `i2c_ctrl_timing_test` | bus timing of the controller vs 4.4.2 / 6.2 / UM10204, measured while traffic runs (base of the speed tests) | `i2c_ip_timing_meas_vseq`, `i2c_ctrl_timing_chk_vseq`, `i2c_ip_ctrl_xfer_vseq` | - |
| VP-CTL-05 | `i2c_ctrl_timing_sm_test` | Standard-mode timing (100 kHz) | `(scenario of i2c_ctrl_timing_test)` | 1x d16 |
| VP-CTL-05 | `i2c_ctrl_timing_fm_test` | Fast-mode timing (400 kHz) | `(scenario of i2c_ctrl_timing_test)` | 1x d16 |
| VP-CTL-05 | `i2c_ctrl_timing_fmp_test` | Fast-mode Plus timing (1 MHz) | `(scenario of i2c_ctrl_timing_test)` | 1x d16 |
| VP-CTL-06 | `i2c_ctrl_cmd_stall_test` | transfer longer than the CMD FIFO: starvation (CMD_STALL, SCL held), CMD_THR-driven refill (6.3, 4.4.3) | `i2c_ctrl_cmd_stall_vseq` | 1x d4,8,16,32 |
| VP-CTL-07 | `i2c_ctrl_crx_full_test` | read longer than CRX with slow software and a stretching target: controller holds SCL, no overflow (4.4.3, F7) | `i2c_ip_ext_dev_cfg_vseq`, `i2c_ctrl_crx_full_vseq` | 1x d4,8,16,32 |
| VP-CTL-08 | `i2c_ctrl_nack_test` | address NACK (write / read, queued transfers flushed), own target never answers, irq for NACK (4.4.4, R4, 6.12) | `i2c_ctrl_addr_nack_vseq`, `i2c_ctrl_own_tgt_vseq`, `i2c_ctrl_nack_irq_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d4,16 |
| VP-CTL-09 | `i2c_ctrl_data_nack_test` | data NACK at every byte position: NACK_DATA, STOP, CMPL, CMD flushed (4.4.4) | `i2c_ip_ext_dev_cfg_vseq`, `i2c_ctrl_data_nack_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d16 |
| VP-CTL-10 | `i2c_ctrl_nakok_test` | NAKOK: refused data / absent address do not abort the transfer, no NACK event (4.4.4) | `i2c_ip_ext_dev_cfg_vseq`, `i2c_ctrl_nakok_vseq` | 1x d16 |
| VP-CTL-11 | `i2c_ctrl_cmderr_test` | command errors E1..E5 in random order (idle / owning the bus), legal READ+NACK then Sr (4.4.5) | `i2c_ctrl_cmderr_vseq`, `i2c_ctrl_nack_rstart_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d4,16 |
| VP-CTL-12 | `i2c_ctrl_abort_test` | CTRL.ABORT idle / waiting for the bus / stalled write / stalled read / read address / read byte in flight, random order (4.4.9, 6.13) | `i2c_ctrl_abort_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d4,16 |
| VP-CTL-13 | `i2c_ctrl_disable_test` | CTRL_EN -> 0 while owning the bus = user abort with STOP (4.4.9) | `i2c_ctrl_disable_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d16 |
| VP-CTL-14 | `i2c_ctrl_start_byte_test` | START byte prefix on every START from idle, not on Sr (4.4.8, 6.7) | `i2c_ctrl_start_byte_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d16 |
| VP-CTL-15 | `i2c_ctrl_gcall_test` | general call with / without NAKOK, Device ID read of an external device (6.8, 6.9) | `i2c_ctrl_gcall_vseq` | 1x d4,16 |
| VP-CTL-16 | `i2c_ctrl_hs_test` | Hs-mode controller: controller code, Hs timing, hs_mode_o / scl_cs_en_o, HS_EN priority, back to F/S (4.4.7, 6.6) | `i2c_ip_hs_cfg_vseq`, `i2c_ctrl_hs_vseq`, `i2c_ctrl_hs_prio_vseq` | 2x d16 |
| VP-CTL-17 | `i2c_ctrl_rstart_chain_test` | corner: one transaction with a chain of repeated STARTs to several targets, read-back through Sr | `i2c_ctrl_rstart_chain_vseq` | 2x d4,16 |
| VP-CTL-18 | `i2c_ctrl_back2back_test` | corner: several complete transactions queued in the CMD FIFO at once (STOP / tBUF / START sequencing) | `i2c_ctrl_back2back_vseq` | 1x d4,16,32 |

### Target (F2, F4, F7, F8, F11, F13; 4.5, 6.10, 6.11)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-TGT-01 | `i2c_tgt_write_test` | target receiver: ADDR0 / ADDR1, STATUS / TSTAT / TRX entries / TADDR / TSTOP / irq_tgt_o, random lengths (4.5, 6.10) | `i2c_tgt_write_vseq` | 1x d4,8,16,32 |
| VP-TGT-02 | `i2c_tgt_rstart_test` | repeated START TRX tags: Sr between own addresses, Sr after a foreign address, write-read (4.5.2) | `i2c_tgt_rstart_vseq` | 1x d4,16 |
| VP-TGT-03 | `i2c_tgt_addr_filter_test` | address filter: foreign address, ADDR0_EN / ADDR1_EN, TGT_EN = 0, EN = 0 (4.5.1) | `i2c_tgt_addr_filter_vseq` | 1x d4,16 |
| VP-TGT-04 | `i2c_tgt_trx_queue_test` | several transactions queued in TRX, TRX_THR interrupt (6.10) | `i2c_tgt_trx_queue_vseq` | 1x d4,8,16,32 |
| VP-TGT-05 | `i2c_tgt_read_test` | target transmitter with preloaded TTX, TTX_DONE, partial read, leftover served next, TTX_FLUSH (4.5.4, 6.11) | `i2c_tgt_read_vseq` | 1x d4,16,32 |
| VP-TGT-06 | `i2c_tgt_tx_req_test` | TTX empty: clock stretching after the address ACK, one TX_REQ, data after the software load (8.5) | `i2c_tgt_tx_req_vseq` | 1x d4,16 |
| VP-TGT-07 | `i2c_tgt_serviced_read_test` | reads longer than TTX refilled by the TX_REQ routine; register read flow S W ptr Sr R (6.11) | `i2c_tgt_serviced_read_vseq` | 1x d4,8,16,32 |
| VP-TGT-08 | `i2c_tgt_stretch_test` | byte-level clock stretching while TRX lacks room, no loss, no TRX_OVF (4.5.4, 8.4) | `i2c_tgt_stretch_vseq` | 1x d4,8,16,32 |
| VP-TGT-09 | `i2c_tgt_nostretch_test` | NOSTRETCH: data / address NACK with TRX full + TRX_OVF, TTX underflow 0xFF + TTX_UNF, never stretches (4.5.3, 4.5.5) | `i2c_tgt_nostretch_rx_vseq`, `i2c_tgt_nostretch_tx_vseq` | 1x d4,8,16,32 |
| VP-TGT-10 | `i2c_tgt_tnack_test` | TNACK: address ACKed, data NACKed, transmit unaffected (5.2.11) | `i2c_tgt_tnack_vseq` | 1x d16 |
| VP-TGT-11 | `i2c_tgt_gcall_test` | general call: GC_EN, GC_SWRST, TSTAT.GC, Sr to the own address after a general call (4.5.1) | `i2c_tgt_gcall_vseq` | 1x d16 |
| VP-TGT-12 | `i2c_tgt_rsvd_addr_test` | reserved addresses / CBUS / 1111_1xx never acknowledged, Hs controller codes -> HS_DET (4.5.1, 4.3.4) | `i2c_tgt_rsvd_addr_vseq` | 1x d16 |
| VP-TGT-13 | `i2c_tgt_addr10_test` | 10-bit addressing not supported (1.4): headers refused, normal traffic right after | `i2c_tgt_addr10_vseq` | 1x d16 |
| VP-TGT-14 | `i2c_tgt_devid_test` | Device ID: 0xF8 / 0xF9 sequence, wrap-around, not armed / foreign / disabled, Sr disarms (4.5.6, 6.9) | `i2c_tgt_devid_vseq` | 2x d16 |
| VP-TGT-15 | `i2c_tgt_rand_addr_test` | corner: random own addresses (whole legal range), writes / serviced reads to both | `i2c_tgt_write_vseq`, `i2c_tgt_serviced_read_vseq` | 2x d4,16 |
| VP-TGT-16 | `i2c_tgt_addr_hold_test` | corner: address (write / read) ACKed while TRX lacks 2 free entries: stretch until the software reads TRX, then TX_REQ for a read (4.5.3, 4.5.4) | `i2c_tgt_addr_hold_vseq` | 2x d4,8,16,32 |
| VP-TGT-17 | `i2c_tgt_combined_test` | corner: combined format: read + Sr write / read, Device ID arming + Sr read, own read + Sr Device ID (4.5.2, 4.5.6) | `i2c_tgt_combined_vseq` | 1x d4,16,32 |
| VP-TGT-18 | `i2c_tgt_devid_corner_test` | corner: Device ID combined formats D1..D11: 0xF8 without address byte, repeated 0xF8, 0xF9 not armed / after a normal frame / DEVID_EN latched 0 at the Sr, repeated 0xF9, reserved ADDR0, no TRX entries (4.5.6) | `i2c_tgt_devid_corner_vseq`, `i2c_tgt_devid_vseq` | 2x d16 |

### Controller + Target (F2, 4.6)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-SYS-01 | `i2c_dual_role_test` | controller and target of the same IP sharing the bus with an external controller (4.6) | `i2c_dual_role_vseq` | 2x d4,16 |

### Mức bus (F6, F7, F10, F14, F15, F19; 4.2, 4.3, 4.10)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-BUS-01 | `i2c_arb_lost_addr_test` | arbitration lost in the address, the winner addresses the DUT target; re-try (4.4.6, 6.13) | `i2c_arb_lost_addr_vseq` | 2x d16 |
| VP-BUS-02 | `i2c_arb_win_test` | the other controller loses against the DUT; clock synchronisation during the address (4.4.6, 4.4.2) | `i2c_arb_win_vseq` | 1x d16 |
| VP-BUS-03 | `i2c_arb_lost_data_test` | arbitration lost in a data byte (same address): no CMPL, CSTAT.ARB_LOST, data of the winner (4.4.6) | `i2c_arb_lost_data_vseq` | 2x d16 |
| VP-BUS-04 | `i2c_clk_sync_test` | another device stretches SCL inside bytes at random bits (clock synchronisation, F6 / F7) | `i2c_clk_sync_vseq` | 2x d16 |
| VP-BUS-05 | `i2c_busclr_test` | bus clear: without CTRL_EN, while the controller is busy, SDA released after k clocks, SDA stuck, SCL stuck + timeout (4.4.10, 6.14) | `i2c_busclr_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d16 |
| VP-BUS-06 | `i2c_scl_timeout_test` | SCL low timeout: controller abort without STOP, target stretch released, TOUT_EN = 0 (4.3.5, 4.4.9, 4.5.9) | `i2c_ip_timeout_cfg_vseq`, `i2c_tout_ctrl_vseq`, `i2c_tout_tgt_vseq`, `i2c_tout_off_vseq`, `i2c_ip_ctrl_xfer_vseq` | 3x d16 |
| VP-BUS-07 | `i2c_bus_err_test` | misplaced START / STOP at random bits: target back to idle, controller abort without STOP (4.3.2, 4.4.9, 4.5.9) | `i2c_bus_err_tgt_vseq`, `i2c_bus_err_ctrl_vseq`, `i2c_ip_ctrl_xfer_vseq`, `i2c_ip_ext_xfer_vseq`, `i2c_ip_trx_collect_vseq` | 2x d4,16 |
| VP-BUS-08 | `i2c_filter_test` | spike filter: SPKLEN_FS boundary (EN = 1 / EN = 0, random length), latency, spikes during a transfer (4.2) | `i2c_filter_spk_vseq`, `i2c_filter_traffic_vseq` | 2x d16 |
| VP-BUS-09 | `i2c_busmon_test` | BUSMON: SCL_F / SDA_F, line override with EN = 0 (bit-bang recovery), ignored with EN = 1 (4.10) | `i2c_busmon_vseq` | 1x d16 |
| VP-BUS-10 | `i2c_bus_busy_test` | bus busy / free detection: IDLE_VAL after enable, S..P, tBUF before bus_free and before the DUT START (4.3.3) | `i2c_bus_busy_vseq` | 1x d16 |
| VP-BUS-11 | `i2c_swrst_test` | SWRST: cleared / kept state of 5.2.1, SWRST in the middle of a transfer, recovery (5.2.1, 6.1.2) | `i2c_swrst_regs_vseq`, `i2c_swrst_active_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d16 |
| VP-BUS-12 | `i2c_en_test` | EN -> 0 during a controller transfer (bus released, FIFO kept); TGT_EN -> 0 inside a transaction (4.1, 4.5.9) | `i2c_en_ctrl_vseq`, `i2c_tgt_en_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d4,16 |
| VP-BUS-13 | `i2c_arb_lost_sr_test` | corner: arbitration lost while sending a repeated START (SDA low when SCL rises, 4.4.6) | `i2c_arb_lost_sr_vseq` | 2x d16 |
| VP-BUS-14 | `i2c_arb_lost_nack_test` | corner: two controllers read the same target, the DUT NACK loses against the other ACK (4.4.6) | `i2c_arb_lost_nack_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d4,16 |
| VP-BUS-15 | `i2c_arb_lost_stop_test` | corner: SDA held low after the STOP release: STOP watchdog = arbitration lost, no hang (4.4.3, 4.4.6) | `i2c_arb_lost_stop_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d16 |
| VP-BUS-16 | `i2c_arb_lost_sp_test` | corner: SCL pulled low by another device during the Sr / STOP setup: arbitration lost, recovery (4.4.6) | `i2c_arb_lost_sp_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d4,16 |
| VP-BUS-17 | `i2c_arb_hold_test` | corner: SCL pulled low by another device inside the START / Sr hold (clock synchronization, no loss), before the START is visible and after the SDA release of the STOP (arbitration lost) (4.4.3, 4.4.6) | `i2c_arb_hold_vseq`, `i2c_ip_ctrl_xfer_vseq` | 3x d4,16 |
| VP-BUS-18 | `i2c_bus_err_ttx_test` | corner: misplaced START / STOP while the target transmits a 1 bit and in the ACK clock of the controller: per-state behaviour, TTX / TRX / events (4.3.2, 4.5.10) | `i2c_bus_err_ttx_vseq`, `i2c_ip_ext_xfer_vseq`, `i2c_ip_trx_collect_vseq` | 3x d4,16 |

### DMA (F18; 4.9, 6.15)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-DMA-01 | `i2c_dma_cmd_test` | DMA SEL = CMD with DMAC flow control: BLOCK / LAST_OKAY, DMA_DONE on irq_o (6.15.1) | `i2c_dma_cmd_vseq` | 1x d4,8,16,32 |
| VP-DMA-02 | `i2c_dma_crx_test` | DMA SEL = CRX: DMAC flow control with DENY of SINGLE, peripheral flow control LAST_BLOCK (6.15.2, 4.9.4) | `i2c_dma_crx_vseq` | 1x d4,8,16,32 |
| VP-DMA-03 | `i2c_dma_trx_test` | DMA SEL = TRX with peripheral flow control: one operation = one transaction (6.15.3) | `i2c_dma_trx_vseq` | 1x d4,8,16,32 |
| VP-DMA-04 | `i2c_dma_ttx_test` | DMA SEL = TTX: the DMAC loads the bytes read by the external controller (6.15.4) | `i2c_dma_ttx_vseq` | 1x d4,8,16,32 |
| VP-DMA-05 | `i2c_dma_blk_test` | DMA_BLK = 0 (SINGLE only, reserved ack = OKAY), DMA_BLK > FIFO_DEPTH clamped (5.2.23, 4.9.3) | `i2c_dma_blk_vseq` | 1x d4,8,16,32 |
| VP-DMA-06 | `i2c_dma_req_hold_test` | a request is never withdrawn (EN = 0, SWRST), SEL / PFC / BLK locked while pending (4.9.2, 5.2.1) | `i2c_dma_req_hold_vseq`, `i2c_ip_ctrl_xfer_vseq` | 1x d4,16 |
| VP-DMA-07 | `i2c_dma_deny_test` | DENY: no new request until the level changes; LAST_SINGLE / LAST_BLOCK of a stopped channel (4.9.2, 4.9.5) | `i2c_dma_deny_vseq` | 1x d4,8,16,32 |
| VP-DMA-08 | `i2c_dma_ack_type_test` | ack types: LAST_* answered LAST_OKAY, reserved ack 2'b11 = OKAY (4.9.3) | `i2c_dma_ack_type_vseq` | 1x d4,16 |
| VP-DMA-09 | `i2c_dma_abort_test` | controller abort (NACK) with SEL = CMD clears DMA_CTRL.EN, recovery (4.4.9, 6.13) | `i2c_dma_abort_vseq` | 1x d4,16 |
| VP-DMA-10 | `i2c_dma_rand_test` | random DMA operations: SEL, PFC, BLK, lengths (+N_ITER) | `i2c_dma_rand_vseq` | 3x d4,16,32 |
| VP-DMA-11 | `i2c_dma_pfc_late_test` | corner: RX peripheral flow control with the LAST entry already in CRX (lpos > BLK, = BLK, < BLK), 4.9.2 | `i2c_dma_pfc_late_vseq` | 1x d4,8,16,32 |
| VP-DMA-12 | `i2c_dma_end_test` | end handshake from the DMAC (dma_end_req_i / dma_end_ack_o, DMA-350 trigger output): ack one cycle later, DMA_STAT.END, END_EN = 0 vs 1, pulse source, request kept at the end event, END_EN lock (4.9.6, 5.2.22, 5.2.24) | `i2c_dma_end_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d4,16,32 |
| VP-DMA-13 | `i2c_dma_clr_test` | DMAC without DENY: request withdrawn by DMA_CTRL.CLR (EN = 0 / EN = 1, IDLE), late ack ignored, ack wins over CLR in the same cycle (4.9.3, 4.9.7) | `i2c_dma_clr_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d4,16 |

### Random (toàn bộ đường dữ liệu)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-RND-01 | `i2c_rand_test` | constrained-random mix of controller and target traffic (+N_ITER, +SPEED) | `i2c_rand_traffic_vseq` | 6x d16; 2x d4,8,32; 1x d16 +SPEED=SM,+N_ITER=12; 1x d16 +N_ITER=150 |
| VP-RND-02 | `i2c_rand_ctrl_test` | random controller-only traffic, long transfers (up to 3 x FIFO_DEPTH) | `i2c_rand_traffic_vseq` | 2x d4,16 |
| VP-RND-03 | `i2c_rand_tgt_test` | random target-only traffic (external controller writes / reads / write-reads) | `i2c_rand_traffic_vseq` | 2x d4,16 |
| VP-RND-04 | `i2c_rand_stretch_test` | random traffic with random target clock stretching: factory type override of the controller transfer | `override: i2c_ip_ctrl_xfer_stretch_vseq` | 2x d4,16 |
| VP-RND-05 | `i2c_rand_reconfig_test` | re-initialisation between bursts: random speed (Sm / Fm / Fm+), own addresses, IDLE_VAL, FIFO thresholds | `i2c_rand_traffic_vseq`, `i2c_ip_init_vseq`, `i2c_ip_fifo_thr_vseq` | 2x d4,16 |

### Q-Channel / low power (F21; 3.8, 4.12, 6.17)

| VP ID | Test | Nội dung kiểm tra (mục spec) | Sequence được test gọi | Regression |
|---|---|---|---|---|
| VP-QCH-01 | `i2c_qch_handshake_test` | IP rảnh: Q_RUN → Q_REQUEST → Q_STOPPED (`QACTIVE = 0`, không tự thức) → Q_EXIT do controller, với `CTRL.EN` 0 / 1, PCLK gate / không gate (4.12.5); truy cập sau khi thoát không có wait state; dừng lâu khi IP enable; `CTRL.EN = 0`: traffic trên bus không đánh thức (4.12.1 – 4.12.3) | `i2c_qch_handshake_vseq`, `i2c_ip_qch_cycle_vseq`, `i2c_ip_ctrl_xfer_vseq` | 2x d16, q0 |
| VP-QCH-02 | `i2c_qch_deny_test` | QDENY khi bận: controller đang truyền, controller giữ SCL (CMD rỗng), target được địa chỉ hóa, bus bận bởi thiết bị khác, bus-idle detection sau enable, SCL bị kéo thấp khi đang dừng (đánh thức) rồi giữ thấp, request DMA đang chờ với `CTRL.EN = 0`; chấp nhận lại khi rảnh (4.12.1, 4.12.2) | `i2c_qch_deny_vseq`, `i2c_ip_qch_cycle_vseq` | 1x d4,16 |
| VP-QCH-03 | `i2c_qch_apb_wake_test` | truy cập APB khi PCLK dừng: `QACTIVE` từ `PSEL`, wait state tới Q_RUN, ghi / đọc thanh ghi, push CMD / TTX và pop CRX / TRX đúng một lần, PSLVERR (offset không định nghĩa, đọc FIFO rỗng + FIFO_ERR); controller đánh thức nhanh và chậm (3.6, 4.12.4) | `i2c_qch_apb_wake_vseq` (2 lần) | 1x d4,16,32 |
| VP-QCH-04 | `i2c_qch_bus_wake_test` | hoạt động bus khi PCLK dừng (`CTRL.EN = 1`): đánh thức trong ngân sách tHD;STA, target write / read / giao dịch giữa thiết bị khác, ở Sm, Fm+, Fm (4.12.3) | `i2c_ip_init_vseq`, `i2c_qch_bus_wake_vseq` (mỗi tốc độ) | 2x d4,16 |
| VP-QCH-05 | `i2c_qch_ctrl_test` | transfer controller bắt đầu từ Q_STOPPED (lệnh CMD đầu tiên đánh thức), QDENY khi transfer đang chạy | `i2c_qch_ctrl_vseq`, `i2c_ip_ctrl_xfer_vseq`, `i2c_ip_qch_cycle_vseq` | 2x d16 |
| VP-QCH-06 | `i2c_qch_dma_test` | `dma_end_req_i` (4 pha) đánh thức IP, `DMA_STAT.END`; DMA TRX (PFC) với PCLK dừng giữa các giao dịch khi `DMA_CTRL.EN = 1` (4.12.1 Q2, 4.12.3) | `i2c_qch_dma_vseq` (`i2c_ip_dma_start_vseq`, `i2c_ip_dma_check_vseq`), `i2c_ip_qch_cycle_vseq` | 1x d4,16,32 |
| VP-QCH-07 | `i2c_qch_reset_test` | trạng thái reset theo `QREQN_RST` (Q_RUN / Q_STOPPED), truy cập đầu tiên từ Q_STOPPED, PRESETn khi đang dừng, hoạt động bình thường sau đó (3.8, 6.1.1) | `i2c_qch_reset_vseq` (`i2c_ip_presetn_vseq`), `i2c_ip_qch_cycle_vseq` | 1x d16, q0 |
| VP-QCH-08 | `i2c_qch_rand_test` | traffic ngẫu nhiên controller / target theo từng đợt + LPI controller nền yêu cầu dừng ngẫu nhiên; scoreboard kiểm toàn bộ dữ liệu (+N_ITER) | `i2c_rand_traffic_vseq`, `i2c_ip_qch_bg_vseq` | 3x d4,16, q0 |

Build `QREQN_RST = 0` (`q0`, LPI controller thoát Q_STOPPED sau reset) cũng chạy `i2c_reg_hw_reset_test`,
`i2c_reg_presetn_test`, `i2c_ctrl_write_read_test`, `i2c_tgt_write_test`, `i2c_dma_trx_test`.

## 5. Review VP (bản này): corner case và random bổ sung

| VP ID | Bổ sung | Lý do |
|---|---|---|
| REG-02 | PRESETn khi IP đang chạy, độ dài xung ngẫu nhiên | 6.1.1 trước đó chỉ kiểm sau reset đầu |
| REG-03 | `uvm_reg_bit_bash_seq` | kiểm từng bit RW / RO theo access policy của RAL |
| REG-04 | Đủ 16 tổ hợp PSTRB trên thanh ghi RW và trên `CMD` | 3.6 trước đó chỉ kiểm từng lane |
| REG-07/10 | Offset unmapped ngẫu nhiên; truy cập thanh ghi ngẫu nhiên (giá trị, PSTRB, PADDR[1:0]) + mirror | lỗi giải mã địa chỉ / strobe |
| INT-02 | Nhiều event + INTR_EN ngẫu nhiên cùng lúc | định tuyến 4 đường ngắt với tổ hợp bất kỳ |
| FIFO-04/05 | CRX_THR / TRX_THR ngẫu nhiên, kiểm ở **mọi** mức FIFO | biên `lvl > thr` |
| CTL-09 | NACK dữ liệu ở mọi vị trí byte 0..3 | biên byte đầu (pointer) |
| CTL-11/12 | E1..E5 và 4 kiểu ABORT theo thứ tự ngẫu nhiên | trạng thái còn sót giữa các lỗi |
| CTL-17 | Chuỗi Sr tới nhiều target trong một transaction | 4.4.1 không giới hạn số Sr |
| CTL-18 | Nhiều transaction đầy đủ xếp sẵn trong CMD FIFO | STOP / tBUF / START liên tiếp không cần SW |
| TGT-05 | Byte TTX còn dư được phục vụ ở lần đọc kế tiếp | 6.11 bước 5 |
| TGT-11 | General call + Sr tới địa chỉ riêng; HW general call (B = 1) thành dữ liệu thường | tag Sr = 11; mục 1.4 |
| TGT-13 | Mỗi header 10-bit bị từ chối được nối tiếp bằng một write có dữ liệu | trạng thái target / monitor sau NACK |
| TGT-15 | Địa chỉ riêng ngẫu nhiên trên toàn dải hợp lệ | bộ so sánh địa chỉ |
| BUS-04/06/07/11 | Vị trí stretch / timeout / START-STOP sai / SWRST ngẫu nhiên theo bit | biên trong byte và ở bit ACK |
| BUS-05 | Thiết bị nhả SDA sau k = 1..8 clock | 4.4.10: luôn 9 xung + STOP |
| BUS-08 | SPKLEN_FS ngẫu nhiên, độ rộng spike ngẫu nhiên < SPKLEN | biên bộ lọc |
| DMA-10 | SEL / PFC / BLK / độ dài ngẫu nhiên | tổ hợp ngoài các ví dụ 4.9.4 |
| RND-02..05 | Random chỉ controller / chỉ target / target stretch ngẫu nhiên (factory override) / cấu hình lại giữa các đợt (tốc độ, địa chỉ, IDLE_VAL, ngưỡng FIFO) | |

Bổ sung từ phân tích code coverage (kịch bản được định nghĩa trong spec nhưng chưa có test; kỳ vọng vẫn lấy từ spec):

| VP ID | Kịch bản | Mục spec |
|---|---|---|
| CTL-12 | `ABORT` khi controller đang chờ bus (controller ngoài giữ bus); `ABORT` trong byte địa chỉ của lệnh đọc (1 byte giả NACK rồi STOP) | 4.4.9 |
| BUS-05 | `BUSCLR` khi controller đang sở hữu bus: bị từ chối, bit tự xóa, transfer tiếp tục | 4.4.10 |
| BUS-13 | Mất arbitration khi đang phát Sr (SDA thấp khi SCL lên) | 4.4.6 |
| BUS-14 | Hai controller cùng đọc: NACK của DUT thua ACK của controller kia; byte đã push vào CRX trước slot NACK | 4.4.6, 4.4.4 |
| BUS-15 | SDA bị giữ thấp sau khi thả để tạo STOP: watchdog STOP → ARB_LOST, không treo | 4.4.3, 4.4.6 |
| BUS-16 | SCL bị thiết bị khác kéo thấp trong lúc DUT phát Sr / STOP (trước TSU_STA / TSU_STO) | 4.4.6 |
| TGT-16 | Địa chỉ (write / read) được ACK khi TRX thiếu 2 chỗ trống: target stretch cho tới khi SW đọc TRX; với read, TX_REQ sau khi entry địa chỉ được push | 4.5.3, 4.5.4 |
| TGT-17 | Combined format: read + Sr write / read, Device ID + Sr read, read + Sr Device ID | 4.5.2, 4.5.6 |
| DMA-11 | RX PFC khi entry LAST đã nằm trong FIFO (kênh bật muộn): `lpos > BLK`, `= BLK`, `< BLK` | 4.9.2 |

Đóng các câu hỏi mở của bản trước (spec 1.4 bổ sung hành vi chi tiết; RTL và reference model cập nhật theo):

| VP ID | Kịch bản | Mục spec |
|---|---|---|
| BUS-17 | SCL bị thiết bị khác kéo thấp **trong** thời gian giữ START / Sr khi SDA đã thấp (vị trí ngẫu nhiên 5..85 % THD_STA) → clock synchronization, transfer hoàn tất, không `ARB_LOST`; SCL đã thấp khi DUT phát START (SDA chưa xuất hiện) → `ARB_LOST` ở `C_STA`; SDA bị giữ thấp qua STOP rồi SCL xuống → `ARB_LOST` ở `C_P_W` | 4.4.3 (bảng), 4.4.6 |
| BUS-18 | START / STOP sai vị trí khi target đang phát bit 1 (clock 1..8 ngẫu nhiên) và ở clock ACK của controller (S sau NACK, P sau ACK): `BUS_ERR` theo `bit_cnt`, entry TRX (`RSTART` / STOP), byte TTX đang phát bị bỏ, không `TTX_DONE`, khôi phục bằng `TTX_FLUSH` | 4.3.2, 4.5.10 |
| TGT-18 | Device ID D1..D11: `0xF8` không có byte địa chỉ rồi Sr `0xF9`, `0xF8` lặp lại hủy armed, byte thừa sau địa chỉ, `0xF9` lặp lại đọc lại từ đầu, `0xF9` khi chưa armed / sau frame thường / khi `DEVID_EN` chốt 0 tại Sr, `ADDR0` thuộc dải reserved, không có entry TRX / `TADDR` / `TSTOP` | 4.5.6 |
| BUS-14 | Byte đọc trước khi thua ở slot NACK nằm trong CRX với `LAST = 1`; test xả CRX như quy trình 6.13 mới | 4.4.6 (5), 6.13 |

Giao diện DMA mở rộng (spec 1.5: cặp `dma_end_req_i` / `dma_end_ack_o` tương đương trigger output của DMA-350,
`DMA_CTRL.END_EN` / `CLR`, `DMA_STAT.BUSY` / `END`). Mô hình DMAC của TB có thêm các chế độ: kết thúc 4 pha / xung /
không có, DMAC không có `DENY`, DMAC không có `LAST_OKAY`, ack muộn sau khi request bị rút:

| VP ID | Kịch bản | Mục spec |
|---|---|---|
| DMA-12 | Kết thúc 4 pha sau `LAST_OKAY` với `END_EN = 0`: ack trễ 1 chu kỳ, chỉ set `DMA_STAT.END`, không thêm `DMA_DONE`; `END_EN = 0` khi `EN = 1`: kênh vẫn bật; DMAC không có `LAST_OKAY` + xung terminal count, `END_EN = 1`: `OKAY` cuối không hoàn tất, xung kết thúc hoàn tất; sự kiện kết thúc khi đang có request (DMAC không DENY): request được giữ tới `CLR`; `END_EN = 1` nhưng `EN = 0`: chỉ ghi nhận; khóa `END_EN` khi `EN = 1` | 4.9.6, 5.2.22, 5.2.24 |
| DMA-13 | DMAC không có `DENY`: request treo vô hạn, cấu hình bị khóa, `CLR` rút request ở chu kỳ sau (`EN = 0` và `EN = 1` — phát lại khi về IDLE), `CLR` ở IDLE không tác dụng; DMAC vẫn phục vụ khi request bị rút: phần tử vào FIFO, ack muộn bị bỏ qua (không `DONE`), handshake chờ ack về 0 (`DMA_STAT.BUSY = 1`, `REQ = 0`); `ack` và `CLR` cùng chu kỳ: `ack` thắng (`DENY` được xử lý); `CLR` với `EN = 1` và ack muộn ở chu kỳ IDLE đầu tiên: chuyển sang ACK ưu tiên hơn request mới, `ack_type` không xử lý, request phát lại sau `ack = 0` | 4.9.3, 4.9.7, 7.2 |
| DMA-10 | Mỗi operation chọn ngẫu nhiên kiểu kết thúc của DMAC (4 pha / xung / không có) với `END_EN = 0` | 4.9.6 |

Kiểm độ nhạy (mutation trên `i2c_dma_ch`): bỏ qua `CLR`, `CLR` thắng `ack`, không chờ ack muộn ở IDLE, hoàn tất
khi `EN = 0`, bỏ qua `END_EN`, `dma_end_ack_o` tổ hợp — cả 6 lỗi đều bị DMA-12 / DMA-13 hoặc SVA phát hiện.

Phát hiện khi chạy các corner case mới: xem `specs/features_and_limitations.md` (bus clear 9 xung có thể không giải
phóng bus khi target rơi đúng slot ACK ở clock STOP; phần mềm phải đọc `BUSMON` và lặp lại theo bước 5 của 6.14 —
`recover()` của TB làm đúng như vậy).

## 6. Ngoài phạm vi / không kiểm được với môi trường hiện tại

| Hạng mục | Lý do |
|---|---|
| Target ở Hs-mode nhận dữ liệu tốc độ 3.4 MHz | VIP I2C không có Hs-mode (chỉ kiểm phát hiện controller code và `hs_mode_o`) |
| Rise/fall time, điện dung bus, current-source pull-up | mô phỏng số, pull-up lý tưởng (t_r = t_f = 0) |
| Arbitration giữa hai controller trong Hs-mode | bit-bang agent chạy timing F/S |
| UFm, 10-bit, CBUS, bridge Hs | không hỗ trợ (1.4); chỉ kiểm hành vi NACK đã định nghĩa |
| Formal, CDC, gate-level simulation | ngoài phạm vi môi trường UVM này (xem `specs/features_and_limitations.md`) |
| Q-Channel: `QREQn` thực sự bất đồng bộ (metastability), glitch trên `QACTIVE` | mô hình clock controller chạy cùng clock nguồn với PCLK; CDC của `QREQn` / `QACTIVE` cần kiểm bằng công cụ CDC |
| Vi phạm ngân sách đánh thức 4.12.3 (clock controller quá chậm) | hành vi khi đó không được đặc tả chi tiết (có thể bỏ lỡ START); không kiểm |
