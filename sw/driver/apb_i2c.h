/*
 * apb_i2c.h — Bare-metal driver for the apb_i2c controller/target IP
 *
 * References
 *   - NXP UM10204 "I2C-bus specification and user manual" (docs/i2c_spec_nxp.pdf)
 *   - apb_i2c architecture spec v1.5 (specs/i2c_architecture_spec.md):
 *     register map (sec. 5) and software sequences (sec. 6)
 *
 * Features
 *   - Timing calculator: Sm / Fm / Fm+ / Hs register values from f_PCLK,
 *     f_SCL and bus rise/fall time (spec 4.11, 6.2; UM10204 tables 10, 12)
 *   - Controller: message-based transfers (Linux i2c_msg style): write, read,
 *     write-then-read with Repeated START, any length (FIFO refilled/drained
 *     on the fly), polled or interrupt driven, blocking or asynchronous,
 *     automatic retry after arbitration loss
 *   - Helpers: register read/write, probe, General Call software reset,
 *     Device ID read, Hs-mode, START byte
 *   - Target: callback based (address / rx byte / stop / tx fill / tx done /
 *     general call reset), from the ISR or from a polling loop
 *   - Recovery: bus clear (hardware, plus BUSMON bit-bang fallback), abort,
 *     software reset
 *   - DMA: channel helpers and command-list builder (DMAC programming is SoC
 *     specific and stays with the application)
 *
 * Execution model
 *   No OS, no heap. All state lives in an i2c_dev_t owned by the caller.
 *   Every hook in i2c_platform_t is optional.
 *   - Polled mode (cfg.use_irq = false): INTR_EN stays 0, the driver reads
 *     INTR_STAT itself. Call i2c_poll() regularly if the target role is used;
 *     blocking controller calls poll internally.
 *   - IRQ mode (cfg.use_irq = true): connect irq_o to the interrupt controller
 *     and call i2c_irq_handler() from the ISR.
 */
#ifndef APB_I2C_H
#define APB_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "apb_i2c_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Register access. Override both macros (e.g. through APB_I2C_IO_HEADER) to   */
/* run the driver on a simulator, a bus bridge or behind an MPU wrapper.     */
/* ------------------------------------------------------------------------- */
#ifdef APB_I2C_IO_HEADER
#include APB_I2C_IO_HEADER
#endif
#ifndef I2C_REG_READ
#define I2C_REG_READ(base, off)       (*(volatile uint32_t *)((base) + (off)))
#endif
#ifndef I2C_REG_WRITE
#define I2C_REG_WRITE(base, off, val) (*(volatile uint32_t *)((base) + (off)) = (uint32_t)(val))
#endif

/* ------------------------------------------------------------------------- */
/* Status codes                                                              */
/* ------------------------------------------------------------------------- */
typedef enum {
    I2C_OK              = 0,
    I2C_ERR_PARAM       = -1,   /* invalid argument / message list           */
    I2C_ERR_BUSY        = -2,   /* a controller transfer is already running  */
    I2C_ERR_NACK_ADDR   = -3,   /* address byte not acknowledged             */
    I2C_ERR_NACK_DATA   = -4,   /* data byte not acknowledged                */
    I2C_ERR_ARB_LOST    = -5,   /* arbitration lost (after all retries)      */
    I2C_ERR_CMD         = -6,   /* CMD_ERR: illegal command sequence         */
    I2C_ERR_ABORTED     = -7,   /* aborted by software                       */
    I2C_ERR_SCL_TIMEOUT = -8,   /* SCL held low longer than TIMEOUT          */
    I2C_ERR_BUS         = -9,   /* misplaced START/STOP                      */
    I2C_ERR_TIMEOUT     = -10,  /* software poll deadline expired            */
    I2C_ERR_BUS_STUCK   = -11,  /* SCL/SDA still low after recovery          */
    I2C_ERR_HW          = -12,  /* IP not found, unsupported, or bad timing  */
    I2C_ERR_DISABLED    = -13   /* role not enabled                          */
} i2c_status_t;

const char *i2c_strerror(int status);

/* ------------------------------------------------------------------------- */
/* Bus speed and timing                                                      */
/* ------------------------------------------------------------------------- */
typedef enum {
    I2C_SPEED_SM  = 0,  /* Standard-mode   <= 100 kbit/s */
    I2C_SPEED_FM  = 1,  /* Fast-mode       <= 400 kbit/s */
    I2C_SPEED_FMP = 2,  /* Fast-mode Plus  <= 1 Mbit/s   */
    I2C_SPEED_HS  = 3   /* High-speed      <= 3.4 Mbit/s (HS_TIM_* registers) */
} i2c_speed_t;

/* Register values in PCLK cycles (spec 5.2.15 / 5.2.16). */
typedef struct {
    uint16_t scl_low;
    uint16_t scl_high;
    uint16_t thd_sta;
    uint16_t tsu_sta;
    uint16_t tsu_sto;
    uint16_t tbuf;      /* F/S only (Hs uses the F/S tBUF) */
    uint16_t thd_dat;
    uint16_t tsu_dat;
    uint8_t  spklen;    /* FILTER.SPKLEN_FS or SPKLEN_HS */
    uint32_t scl_hz;    /* resulting SCL frequency, including tr + tf */
} i2c_timing_t;

/*
 * Compute timing register values (spec 6.2 formulas, UM10204 table 10/12 limits).
 *   pclk_hz : PCLK frequency
 *   scl_hz  : wanted SCL frequency, 0 = maximum of the mode
 *   tr_ns   : bus rise time, 0 = UM10204 maximum of the mode
 *   tf_ns   : bus fall time, 0 = UM10204 maximum of the mode
 * The real SCL frequency is never above scl_hz and never above the mode
 * limit; it is lower when PCLK is too slow to meet the minimum tLOW/tHIGH.
 * Returns I2C_ERR_HW when the IP cannot meet the mode at this PCLK (data
 * hold time window, or a value that does not fit in 16 bits).
 */
int i2c_calc_timing(uint32_t pclk_hz, i2c_speed_t speed, uint32_t scl_hz,
                    uint32_t tr_ns, uint32_t tf_ns, i2c_timing_t *t);

/* ------------------------------------------------------------------------- */
/* Platform hooks (all optional)                                             */
/* ------------------------------------------------------------------------- */
typedef struct {
    /* Free running microsecond counter. With it, every timeout below is in
     * microseconds; without it, timeouts count poll-loop iterations.      */
    uint32_t (*now_us)(void);
    /* Busy delay, needed only by i2c_bus_clear_bitbang().                 */
    void     (*delay_us)(uint32_t us);
    /* Called once per iteration of every wait loop: WFI, watchdog kick,
     * servicing another device ...                                       */
    void     (*idle)(void *arg);
    void      *idle_arg;
    /* Critical section against i2c_irq_handler() (IRQ mode only).         */
    uint32_t (*lock)(void);
    void     (*unlock)(uint32_t key);
} i2c_platform_t;

/* ------------------------------------------------------------------------- */
/* Messages (controller)                                                     */
/* ------------------------------------------------------------------------- */
#define I2C_M_RD          0x0001u  /* read len bytes into buf                    */
#define I2C_M_NOSTART     0x0002u  /* continue the previous write message: no Sr,
                                      no address (e.g. header + payload buffers) */
#define I2C_M_IGNORE_NAK  0x0004u  /* write: do not abort on NACK (CMD.NAKOK)    */

typedef struct {
    uint8_t  addr;    /* 7-bit address */
    uint16_t flags;   /* I2C_M_*       */
    uint32_t len;     /* bytes; reads need len >= 1 */
    uint8_t *buf;
} i2c_msg_t;

/*
 * Message rules (map 1:1 on the CMD encoding of spec 4.4.1 and the command
 * rules E1..E5 of spec 4.4.5):
 *   - every message starts with S (first) or Sr (others), except I2C_M_NOSTART
 *   - the last byte of a read message is NACKed (controller-receiver rule)
 *   - STOP follows the last byte of the last message
 *   - a write message with len = 0 sends only the address byte (probe / Sr)
 */

/* ------------------------------------------------------------------------- */
/* Target callbacks (all optional)                                           */
/* ------------------------------------------------------------------------- */
typedef struct {
    /* Address byte acknowledged: S (repeated = false) or Sr (repeated = true).
     * addr7 = 0 with read = false is a General Call.                         */
    void   (*on_start)(void *ctx, uint8_t addr7, bool read, bool repeated);
    /* Data byte received (target-receiver).                                  */
    void   (*on_rx)(void *ctx, uint8_t byte);
    /* STOP ending a transaction in which this target was addressed.          */
    void   (*on_stop)(void *ctx);
    /* Target-transmitter needs data: copy up to max bytes into buf and return
     * the count. Returning 0 sends 0xFF (the bus must not stay stretched).  */
    size_t (*on_tx)(void *ctx, uint8_t *buf, size_t max);
    /* Controller NACKed the last byte read. unsent = bytes handed over by
     * on_tx that were never transmitted (they are flushed).                  */
    void   (*on_tx_done)(void *ctx, uint32_t unsent);
    /* General Call + 0x06 received. The IP does not reset itself.            */
    void   (*on_gc_swrst)(void *ctx);
    /* TRX_OVF, TTX_UNF, BUS_ERR or SCL_TOUT (INTR_STAT bits). On BUS_ERR or
     * SCL_TOUT the frame in progress is incomplete: discard it.              */
    void   (*on_error)(void *ctx, uint32_t intr_bits);
} i2c_target_ops_t;

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */
#define I2C_ADDR_NONE  0xFFu

typedef struct {
    uint32_t    pclk_hz;
    i2c_speed_t speed;          /* SM / FM / FMP */
    uint32_t    scl_hz;         /* 0 = maximum of the mode */
    uint16_t    rise_ns;        /* 0 = UM10204 maximum     */
    uint16_t    fall_ns;        /* 0 = UM10204 maximum     */

    bool        controller;     /* CTRL.CTRL_EN */
    bool        target;         /* CTRL.TGT_EN  */

    /* Target role */
    uint8_t     tgt_addr[2];    /* 7-bit addresses or I2C_ADDR_NONE */
    uint32_t    tgt_cfg;        /* TCFG bits: GC_EN, NOSTRETCH, TNACK, DEVID_EN */
    uint32_t    devid;          /* 24-bit Device ID, used with TCFG.DEVID_EN */
    const i2c_target_ops_t *tgt_ops;
    void       *tgt_ctx;

    /* Hs-mode (controller): HS_TIM_* are programmed when hs_scl_hz != 0 */
    uint32_t    hs_scl_hz;
    uint8_t     hs_mcode;       /* HSCFG.MCODE, 1..7 (unique per Hs controller) */
    bool        hs_enable;      /* CTRL.HS_EN at start-up */

    uint32_t    scl_timeout_us; /* hardware SCL-low timeout, 0 = disabled (SMBus: 25000..35000) */
    uint32_t    bus_idle_ns;    /* BUSIDLE, 0 = auto (2 SCL periods); must exceed the
                                   longest tHIGH of every controller on the bus */
    uint32_t    timeout;        /* blocking-call deadline (us with now_us, else iterations);
                                   0 = wait forever */
    uint8_t     arb_retries;    /* blocking calls: retries after ARB_LOST */
    bool        use_irq;        /* see "Execution model" above */
    const i2c_platform_t *plat;
} i2c_config_t;

/* Fill cfg with defaults: Fm 400 kHz, controller only, polled, 35 ms SCL timeout. */
void i2c_config_default(i2c_config_t *cfg, uint32_t pclk_hz);

/* ------------------------------------------------------------------------- */
/* Device                                                                    */
/* ------------------------------------------------------------------------- */
typedef void (*i2c_xfer_cb_t)(void *ctx, int status);
typedef void (*i2c_dma_cb_t)(void *ctx, uint32_t sel, uint32_t dma_stat);

typedef struct {
    uint32_t bus_err;
    uint32_t scl_tout;
    uint32_t fifo_err;
    uint32_t trx_ovf;
    uint32_t ttx_unf;
    uint32_t arb_lost;
} i2c_stats_t;

/* Command generator: walks a message list and yields CMD words. */
typedef struct {
    const i2c_msg_t *msgs;
    uint32_t         n;
    uint32_t         msg;     /* current message           */
    uint32_t         idx;     /* next byte of that message */
    bool             addr;    /* its address byte is still to be sent */
} i2c_cmd_iter_t;

/* Driver state: treat as opaque. */
typedef struct i2c_dev {
    uintptr_t               base;
    const i2c_platform_t   *plat;
    uint32_t                ctrl;       /* shadow of the CTRL RW bits         */
    volatile uint32_t       mask;       /* enabled sources (= INTR_EN in IRQ mode) */
    uint32_t                fifo_depth;
    uint32_t                timeout;
    uint8_t                 arb_retries;
    bool                    use_irq;
    bool                    hs_ready;   /* HS_TIM_* programmed                */

    struct {
        const i2c_msg_t    *msgs;
        uint32_t            nmsgs;
        i2c_cmd_iter_t      it;         /* command generator position         */
        uint32_t            rx_msg;     /* receive position                   */
        uint32_t            rx_idx;
        uint32_t            rx_expect;
        uint32_t            rx_count;
        uint32_t            cstat;      /* accumulated CSTAT                  */
        bool                stop_feed;  /* abort seen: push no more commands  */
        volatile bool       forced;     /* software abort/timeout in progress */
        volatile bool       busy;
        volatile int        result;
        i2c_xfer_cb_t       cb;
        void               *cb_ctx;
    } x;

    const i2c_target_ops_t *tgt_ops;
    void                   *tgt_ctx;
    i2c_dma_cb_t            dma_cb;
    void                   *dma_ctx;
    i2c_stats_t             stats;
} i2c_dev_t;

/* ------------------------------------------------------------------------- */
/* Init / reset / power-down (spec 6.1, 6.2, 6.16)                           */
/* ------------------------------------------------------------------------- */
int  i2c_init(i2c_dev_t *dev, uintptr_t base, const i2c_config_t *cfg);
int  i2c_deinit(i2c_dev_t *dev);
/* CTRL.SWRST, then re-enable with the current configuration (spec 6.1.2).
 * Stop any DMA channel first.                                               */
void i2c_soft_reset(i2c_dev_t *dev);
uint32_t i2c_version(const i2c_dev_t *dev);

/* ------------------------------------------------------------------------- */
/* Interrupt / polling entry points                                          */
/* ------------------------------------------------------------------------- */
void i2c_irq_handler(i2c_dev_t *dev);   /* IRQ mode: call from the irq_o ISR  */
void i2c_poll(i2c_dev_t *dev);          /* polled mode: call from the main loop */

/* ------------------------------------------------------------------------- */
/* Controller                                                                */
/* ------------------------------------------------------------------------- */
/* Blocking. Do not call from the I2C ISR. Retries cfg.arb_retries times on ARB_LOST. */
int  i2c_master_transfer(i2c_dev_t *dev, const i2c_msg_t *msgs, uint32_t n);
/* Non-blocking: cb(ctx, status) runs from i2c_irq_handler()/i2c_poll().
 * msgs and their buffers must stay valid until then.                        */
int  i2c_master_transfer_async(i2c_dev_t *dev, const i2c_msg_t *msgs, uint32_t n,
                               i2c_xfer_cb_t cb, void *ctx);
bool i2c_master_busy(const i2c_dev_t *dev);
/* Abort the running transfer (CTRL.ABORT, spec 4.4.9, 6.13) and wait for the
 * controller to release the bus. The transfer ends with I2C_ERR_ABORTED.    */
int  i2c_master_abort(i2c_dev_t *dev);

int  i2c_master_write(i2c_dev_t *dev, uint8_t addr, const uint8_t *data, uint32_t len);
int  i2c_master_read(i2c_dev_t *dev, uint8_t addr, uint8_t *data, uint32_t len);
int  i2c_master_write_read(i2c_dev_t *dev, uint8_t addr,
                           const uint8_t *wdata, uint32_t wlen,
                           uint8_t *rdata, uint32_t rlen);
/* S addr W reg[0..reg_len-1] data[...] P */
int  i2c_reg_write(i2c_dev_t *dev, uint8_t addr, const uint8_t *reg, uint32_t reg_len,
                   const uint8_t *data, uint32_t len);
/* S addr W reg[...] Sr addr R data[...] P  (spec 6.5) */
int  i2c_reg_read(i2c_dev_t *dev, uint8_t addr, const uint8_t *reg, uint32_t reg_len,
                  uint8_t *data, uint32_t len);
/* S addr W P : I2C_OK if a device acknowledges addr */
int  i2c_probe(i2c_dev_t *dev, uint8_t addr);
/* S 0x00 A 0x06 A P (spec 6.8, UM10204 3.1.14) */
int  i2c_general_call_reset(i2c_dev_t *dev);
/* S 0xF8 A addr A Sr 0xF9 A id2 A id1 A id0 Ā P (spec 6.9, UM10204 3.1.17).
 * I2C_ERR_NACK_ADDR / NACK_DATA = no Device ID support / no such device.    */
int  i2c_read_device_id(i2c_dev_t *dev, uint8_t addr, uint32_t *devid);

/* Controller must be idle. Hs timing must have been set by i2c_init(). */
int  i2c_set_hs_mode(i2c_dev_t *dev, bool enable);
int  i2c_set_start_byte(i2c_dev_t *dev, bool enable);

/* ------------------------------------------------------------------------- */
/* Target                                                                    */
/* ------------------------------------------------------------------------- */
void i2c_target_set_ops(i2c_dev_t *dev, const i2c_target_ops_t *ops, void *ctx);
/* Change addresses / TCFG at run time; effective from the next START.       */
void i2c_target_set_addr(i2c_dev_t *dev, uint8_t addr0, uint8_t addr1);
void i2c_target_set_cfg(i2c_dev_t *dev, uint32_t tcfg);

/* ------------------------------------------------------------------------- */
/* Bus recovery (spec 6.13, 6.14; UM10204 3.1.16)                            */
/* ------------------------------------------------------------------------- */
/* Hardware bus clear: 9 SCL pulses + STOP. I2C_OK when SCL = SDA = 1 after. */
int  i2c_bus_clear(i2c_dev_t *dev);
/* Software fallback through BUSMON override (needs plat->delay_us).         */
int  i2c_bus_clear_bitbang(i2c_dev_t *dev);
/* Inspect the lines and run the right recovery step:
 *   SDA low, SCL high -> bus clear (hardware, then bit-bang)
 *   SCL low           -> I2C_ERR_BUS_STUCK (reset/power-cycle the culprit)  */
int  i2c_recover(i2c_dev_t *dev);
/* BUSMON: bit0 = SCL, bit1 = SDA (filtered line levels) */
uint32_t i2c_bus_lines(const i2c_dev_t *dev);

/* ------------------------------------------------------------------------- */
/* DMA (spec 4.9, 6.15). The DMAC channel itself is programmed by the caller. */
/* ------------------------------------------------------------------------- */
typedef enum {
    I2C_DMA_SEL_CMD = 0,  /* memory -> I2C_BASE + 0x01C (32-bit commands) */
    I2C_DMA_SEL_CRX = 1,  /* I2C_BASE + 0x020 -> memory                   */
    I2C_DMA_SEL_TTX = 2,  /* memory -> I2C_BASE + 0x024                   */
    I2C_DMA_SEL_TRX = 3   /* I2C_BASE + 0x028 -> memory                   */
} i2c_dma_sel_t;

#define I2C_DMA_F_PFC     I2C_DMA_CTRL_PFC     /* peripheral flow control (CRX/TRX) */
#define I2C_DMA_F_END_EN  I2C_DMA_CTRL_END_EN  /* dma_end_req_i completes the channel */

/* Program SEL/PFC/END_EN/BLK and enable the channel (enable the DMAC
 * channel first). Fails with I2C_ERR_BUSY if the channel is not idle.       */
int  i2c_dma_start(i2c_dev_t *dev, i2c_dma_sel_t sel, uint32_t blk, uint32_t flags);
/* EN = 0 + CLR, wait DMA_STAT.BUSY = 0. Stop the DMAC channel first.         */
int  i2c_dma_stop(i2c_dev_t *dev);
/* DMA_DONE callback (from i2c_irq_handler()/i2c_poll()); NULL disables it.  */
void i2c_dma_set_callback(i2c_dev_t *dev, i2c_dma_cb_t cb, void *ctx);
/* Build the 32-bit CMD words of a message list for a SEL = CMD DMA channel
 * (spec 6.15.1). Returns the number of words, or a negative error code.
 * The read data of such a transfer arrives in CRX (CPU or a 2nd operation). */
int  i2c_build_cmds(const i2c_msg_t *msgs, uint32_t n, uint32_t *out, uint32_t max_words);

#ifdef __cplusplus
}
#endif

#endif /* APB_I2C_H */
