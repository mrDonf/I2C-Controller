/*
 * apb_i2c.c — Bare-metal driver for the apb_i2c controller/target IP
 *
 * Section numbers "spec x.y" refer to specs/i2c_architecture_spec.md (v1.5),
 * "UM x.y" to NXP UM10204 (docs/i2c_spec_nxp.pdf).
 */
#include "apb_i2c.h"

#define RD(d, off)      I2C_REG_READ((d)->base, (off))
#define WR(d, off, v)   I2C_REG_WRITE((d)->base, (off), (v))

/* Sources armed only while a controller transfer runs */
#define SRC_XFER   (I2C_INTR_CMPL | I2C_INTR_NACK | I2C_INTR_ARB_LOST | I2C_INTR_CMD_ERR)
#define SRC_FEED   (I2C_INTR_CMD_THR | I2C_INTR_CRX_THR)
/* Sources armed for the whole life of the device */
#define SRC_BASE   (I2C_INTR_SCL_TOUT | I2C_INTR_BUS_ERR | I2C_INTR_FIFO_ERR)

/* Abort causes that are NOT followed by CMPL (spec 4.4.6, 4.4.9) */
#define CSTAT_NO_CMPL  (I2C_CSTAT_ARB_LOST | I2C_CSTAT_SCL_TOUT | I2C_CSTAT_BUS_ERR)

/* Bound for waits that finish within a few PCLK cycles by spec */
#define SHORT_WAIT     1000u

#define NS_PER_S       1000000000ull

/* ========================================================================= */
/* Small helpers                                                             */
/* ========================================================================= */
static uint32_t lock(const i2c_dev_t *d)
{
    return (d->plat != NULL && d->plat->lock != NULL) ? d->plat->lock() : 0u;
}

static void unlock(const i2c_dev_t *d, uint32_t key)
{
    if (d->plat != NULL && d->plat->unlock != NULL) {
        d->plat->unlock(key);
    }
}

static void idle(const i2c_dev_t *d)
{
    if (d->plat != NULL && d->plat->idle != NULL) {
        d->plat->idle(d->plat->idle_arg);
    }
}

typedef struct {
    uint32_t t0;
    uint32_t n;
} deadline_t;

static void dl_start(const i2c_dev_t *d, deadline_t *dl)
{
    dl->n  = 0u;
    dl->t0 = (d->plat != NULL && d->plat->now_us != NULL) ? d->plat->now_us() : 0u;
}

/* limit = 0: never expires */
static bool dl_expired(const i2c_dev_t *d, deadline_t *dl, uint32_t limit)
{
    if (limit == 0u) {
        return false;
    }
    if (d->plat != NULL && d->plat->now_us != NULL) {
        return (uint32_t)(d->plat->now_us() - dl->t0) >= limit;
    }
    return ++dl->n >= limit;
}

/* Update the enabled-source mask (and INTR_EN in IRQ mode). */
static void mask_update(i2c_dev_t *d, uint32_t set, uint32_t clr)
{
    uint32_t k = lock(d);
    d->mask = (d->mask & ~clr) | set;
    if (d->use_irq) {
        WR(d, I2C_INTR_EN, d->mask);
    }
    unlock(d, k);
}

/* Write a W1SC command bit of CTRL, keeping the RW bits (spec 5.2.1). */
static void ctrl_cmd(i2c_dev_t *d, uint32_t bits)
{
    WR(d, I2C_CTRL, d->ctrl | bits);
}

static uint32_t fifo_lvl(i2c_dev_t *d, unsigned shift)
{
    return I2C_FIFO_LVL(RD(d, I2C_FIFO_STAT), shift);
}

/* 7-bit address usable as a target address (not reserved, spec 4.5.1) */
static bool addr_assignable(uint8_t a)
{
    return (a >= 0x08u) && (a <= 0x77u);
}

static uint32_t taddr_value(uint8_t a0, uint8_t a1)
{
    uint32_t v = 0u;
    if (a0 != I2C_ADDR_NONE) {
        v |= ((uint32_t)a0 << I2C_TADDR_ADDR0_SHIFT) | I2C_TADDR_ADDR0_EN;
    }
    if (a1 != I2C_ADDR_NONE) {
        v |= ((uint32_t)a1 << I2C_TADDR_ADDR1_SHIFT) | I2C_TADDR_ADDR1_EN;
    }
    return v;
}

const char *i2c_strerror(int status)
{
    switch (status) {
    case I2C_OK:              return "ok";
    case I2C_ERR_PARAM:       return "invalid parameter";
    case I2C_ERR_BUSY:        return "busy";
    case I2C_ERR_NACK_ADDR:   return "address NACK";
    case I2C_ERR_NACK_DATA:   return "data NACK";
    case I2C_ERR_ARB_LOST:    return "arbitration lost";
    case I2C_ERR_CMD:         return "command error";
    case I2C_ERR_ABORTED:     return "aborted";
    case I2C_ERR_SCL_TIMEOUT: return "SCL low timeout";
    case I2C_ERR_BUS:         return "bus error";
    case I2C_ERR_TIMEOUT:     return "software timeout";
    case I2C_ERR_BUS_STUCK:   return "bus stuck";
    case I2C_ERR_HW:          return "hardware/timing error";
    case I2C_ERR_DISABLED:    return "role disabled";
    default:                  return "unknown";
    }
}

/* ========================================================================= */
/* Timing calculator (spec 4.11, 6.2)                                        */
/* ========================================================================= */
typedef struct {
    uint32_t f_max;     /* Hz */
    uint32_t t_low;     /* all times in ns, UM10204 tables 10 and 12 */
    uint32_t t_high;
    uint32_t thd_sta;
    uint32_t tsu_sta;
    uint32_t tsu_sto;
    uint32_t tbuf;
    uint32_t tsu_dat;
    uint32_t tvd_dat;   /* F/S: tVD;DAT max.  Hs: tHD;DAT max */
    uint32_t tr_scl;
    uint32_t tf_scl;
    uint32_t tr_sda;
    uint32_t tsp;
} mode_limits_t;

static const mode_limits_t k_limits[4] = {
    /*            f_max  tLOW tHIGH tHDSTA tSUSTA tSUSTO tBUF tSUDAT tVD  tr    tf   trSDA tSP */
    /* Sm  */ {  100000u, 4700u, 4000u, 4000u, 4700u, 4000u, 4700u, 250u, 3450u, 1000u, 300u, 1000u, 50u },
    /* Fm  */ {  400000u, 1300u,  600u,  600u,  600u,  600u, 1300u, 100u,  900u,  300u, 300u,  300u, 50u },
    /* Fm+ */ { 1000000u,  500u,  260u,  260u,  260u,  260u,  500u,  50u,  450u,  120u, 120u,  120u, 50u },
    /* Hs  */ { 3400000u,  160u,   60u,  160u,  160u,  160u,    0u,  10u,   70u,   40u,  40u,   80u, 10u },
};

static uint32_t cyc_ceil(uint64_t ns, uint32_t pclk_hz)
{
    return (uint32_t)((ns * pclk_hz + NS_PER_S - 1u) / NS_PER_S);
}

static uint32_t cyc_floor(uint64_t ns, uint32_t pclk_hz)
{
    return (uint32_t)((ns * pclk_hz) / NS_PER_S);
}

static uint32_t sub_sat(uint32_t a, uint32_t b)
{
    return (a > b) ? a - b : 0u;
}

int i2c_calc_timing(uint32_t pclk_hz, i2c_speed_t speed, uint32_t scl_hz,
                    uint32_t tr_ns, uint32_t tf_ns, i2c_timing_t *t)
{
    if (t == NULL || pclk_hz == 0u || (unsigned)speed > (unsigned)I2C_SPEED_HS) {
        return I2C_ERR_PARAM;
    }
    const mode_limits_t *m = &k_limits[speed];
    const bool hs = (speed == I2C_SPEED_HS);

    if (scl_hz == 0u || scl_hz > m->f_max) {
        scl_hz = m->f_max;
    }
    const uint32_t tr    = (tr_ns != 0u) ? tr_ns : m->tr_scl;
    const uint32_t tf    = (tf_ns != 0u) ? tf_ns : m->tf_scl;
    const uint32_t trsda = (tr_ns != 0u) ? tr_ns : m->tr_sda;

    /* Spike filter and input latency: SPKLEN = ceil(tSP/T), LAT = SPKLEN + 3 */
    uint32_t spk = cyc_ceil(m->tsp, pclk_hz);
    if (spk == 0u) {
        spk = 1u;
    }
    if (spk > 0xFFu) {
        return I2C_ERR_HW;
    }
    const uint32_t lat = spk + 3u;

    /* Data hold, real value (LAT + HD + 1)*T (spec 4.4.2). */
    uint32_t hd;
    if (hs) {
        /* Hs: tHD;DAT <= 70 ns (UM table 12); use the smallest value. */
        hd = 1u;
        uint64_t hold_ns = ((uint64_t)(lat + hd + 1u) * NS_PER_S + pclk_hz - 1u) / pclk_hz;
        if (hold_ns > m->tvd_dat) {
            return I2C_ERR_HW;           /* PCLK too slow for Hs-mode */
        }
    } else {
        /* >= 300 ns to bridge the undefined region of the SCL falling edge
         * (UM table 10 note 3), <= tVD;DAT(max) - tf. */
        if (tf >= m->tvd_dat) {
            return I2C_ERR_PARAM;
        }
        uint32_t lim = cyc_floor(m->tvd_dat - tf, pclk_hz);
        if (lim < lat + 2u) {
            return I2C_ERR_HW;           /* PCLK too slow for this mode */
        }
        hd = sub_sat(cyc_ceil(300u, pclk_hz), lat + 1u);
        if (hd == 0u) {
            hd = 1u;
        }
        if (hd > lim - lat - 1u) {
            hd = lim - lat - 1u;
        }
    }

    /* SCL low/high: tLOW ~ (L+LAT)T + tr, tHIGH ~ (H+LAT)T + tf (spec 4.4.2) */
    uint32_t l_min = sub_sat(cyc_ceil(m->t_low, pclk_hz), lat);
    uint32_t h_min = sub_sat(cyc_ceil(m->t_high, pclk_hz), lat);
    if (l_min < hd + 2u) {
        l_min = hd + 2u;                 /* programming rule L >= HD + 2 */
    }
    if (h_min < 1u) {
        h_min = 1u;
    }
    uint32_t period_ns = (uint32_t)((NS_PER_S + scl_hz - 1u) / scl_hz);
    uint32_t total = sub_sat(cyc_ceil(sub_sat(period_ns, tr + tf), pclk_hz), 2u * lat);
    if (total < l_min + h_min) {
        total = l_min + h_min;
    }
    /* Split L:H in the ratio of the minimum tLOW:tHIGH of the mode. */
    uint32_t l = (uint32_t)(((uint64_t)total * m->t_low + (m->t_low + m->t_high) - 1u) /
                            (m->t_low + m->t_high));
    if (l < l_min) {
        l = l_min;
    }
    uint32_t h = sub_sat(total, l);
    if (h < h_min) {
        h = h_min;
    }

    uint32_t thd_sta = cyc_ceil(m->thd_sta, pclk_hz);
    uint32_t tsu_sta = cyc_ceil(m->tsu_sta, pclk_hz);
    uint32_t tsu_sto = cyc_ceil(m->tsu_sto, pclk_hz);
    uint32_t tbuf    = cyc_ceil(m->tbuf, pclk_hz);
    uint32_t tsu_dat = cyc_ceil((uint64_t)m->tsu_dat + trsda, pclk_hz);

    if (l > 0xFFFFu || h > 0xFFFFu || thd_sta > 0xFFFFu || tsu_sta > 0xFFFFu ||
        tsu_sto > 0xFFFFu || tbuf > 0xFFFFu || tsu_dat > 0xFFFFu) {
        return I2C_ERR_HW;               /* PCLK too fast for 16-bit counters */
    }

    t->scl_low  = (uint16_t)l;
    t->scl_high = (uint16_t)h;
    t->thd_sta  = (uint16_t)(thd_sta ? thd_sta : 1u);
    t->tsu_sta  = (uint16_t)(tsu_sta ? tsu_sta : 1u);
    t->tsu_sto  = (uint16_t)(tsu_sto ? tsu_sto : 1u);
    t->tbuf     = (uint16_t)tbuf;
    t->thd_dat  = (uint16_t)hd;
    t->tsu_dat  = (uint16_t)(tsu_dat ? tsu_dat : 1u);
    t->spklen   = (uint8_t)spk;

    uint64_t period_ps = ((uint64_t)(l + h + 2u * lat) * 1000000000000ull) / pclk_hz +
                         (uint64_t)(tr + tf) * 1000u;
    t->scl_hz = (uint32_t)(1000000000000ull / period_ps);
    return I2C_OK;
}

/* ========================================================================= */
/* Command generation (spec 4.4.1, 4.4.5)                                    */
/* ========================================================================= */
static void iter_init(i2c_cmd_iter_t *it, const i2c_msg_t *msgs, uint32_t n)
{
    it->msgs = msgs;
    it->n    = n;
    it->msg  = 0u;
    it->idx  = 0u;
    it->addr = true;
}

static bool iter_next(i2c_cmd_iter_t *it, uint32_t *cmd)
{
    while (it->msg < it->n) {
        const i2c_msg_t *m = &it->msgs[it->msg];
        const bool last = (it->msg + 1u == it->n);
        const bool rd   = (m->flags & I2C_M_RD) != 0u;
        const uint32_t nak = (m->flags & I2C_M_IGNORE_NAK) ? I2C_CMD_NAKOK : 0u;

        if (it->addr) {
            it->addr = false;
            if ((m->flags & I2C_M_NOSTART) == 0u) {
                uint32_t c = I2C_CMD_START | ((uint32_t)m->addr << 1) | (rd ? 1u : 0u) | nak;
                if (m->len == 0u && last) {
                    c |= I2C_CMD_STOP;
                }
                *cmd = c;
                return true;
            }
        }
        if (it->idx < m->len) {
            const uint32_t i = it->idx++;
            const bool last_byte = (i + 1u == m->len);
            uint32_t c;
            if (rd) {
                /* Last byte of a read is NACKed; then Sr (E4) or P follows. */
                c = I2C_CMD_READ;
                if (last_byte) {
                    c |= I2C_CMD_NACK | (last ? I2C_CMD_STOP : 0u);
                }
            } else {
                c = (uint32_t)m->buf[i] | nak;
                if (last_byte && last) {
                    c |= I2C_CMD_STOP;
                }
            }
            *cmd = c;
            return true;
        }
        it->msg++;
        it->idx  = 0u;
        it->addr = true;
    }
    return false;
}

static bool iter_more(const i2c_cmd_iter_t *it)
{
    i2c_cmd_iter_t tmp = *it;
    uint32_t c;
    return iter_next(&tmp, &c);
}

/* Reject lists that would violate the command rules E1..E5 (spec 4.4.5). */
static int msgs_check(const i2c_msg_t *msgs, uint32_t n, uint32_t *rx_total)
{
    if (msgs == NULL || n == 0u) {
        return I2C_ERR_PARAM;
    }
    uint32_t rx = 0u;
    for (uint32_t i = 0u; i < n; i++) {
        const i2c_msg_t *m = &msgs[i];
        if (m->addr > 0x7Fu || (m->len != 0u && m->buf == NULL)) {
            return I2C_ERR_PARAM;
        }
        if ((m->flags & I2C_M_RD) != 0u) {
            if (m->len == 0u || (m->flags & I2C_M_NOSTART) != 0u || rx + m->len < rx) {
                return I2C_ERR_PARAM;
            }
            rx += m->len;
        } else if ((m->flags & I2C_M_NOSTART) != 0u) {
            if (i == 0u || (msgs[i - 1u].flags & I2C_M_RD) != 0u || m->len == 0u) {
                return I2C_ERR_PARAM;
            }
        }
    }
    *rx_total = rx;
    return I2C_OK;
}

int i2c_build_cmds(const i2c_msg_t *msgs, uint32_t n, uint32_t *out, uint32_t max_words)
{
    uint32_t rx;
    int rc = msgs_check(msgs, n, &rx);
    if (rc != I2C_OK) {
        return rc;
    }
    if (out == NULL) {
        return I2C_ERR_PARAM;
    }
    i2c_cmd_iter_t it;
    iter_init(&it, msgs, n);
    uint32_t cnt = 0u;
    uint32_t c;
    while (iter_next(&it, &c)) {
        if (cnt >= max_words || cnt >= 0x7FFFFFFFu) {
            return I2C_ERR_PARAM;
        }
        out[cnt++] = c;
    }
    return (int)cnt;
}

/* ========================================================================= */
/* Controller engine                                                         */
/* ========================================================================= */
static void rx_store(i2c_dev_t *d, uint32_t entry)
{
    while (d->x.rx_msg < d->x.nmsgs) {
        const i2c_msg_t *m = &d->x.msgs[d->x.rx_msg];
        if ((m->flags & I2C_M_RD) != 0u && d->x.rx_idx < m->len) {
            m->buf[d->x.rx_idx++] = (uint8_t)(entry & I2C_CRX_DATA_MASK);
            d->x.rx_count++;
            return;
        }
        d->x.rx_msg++;
        d->x.rx_idx = 0u;
    }
    /* byte beyond the expected count: dropped (cannot happen without abort) */
}

/* Push as many commands as the CMD FIFO can take. */
static void ctrl_feed(i2c_dev_t *d)
{
    if (d->x.stop_feed) {
        return;
    }
    uint32_t lvl  = fifo_lvl(d, I2C_FIFO_CMD_SHIFT);
    uint32_t room = sub_sat(d->fifo_depth, lvl);
    uint32_t c;
    while (room > 0u && iter_next(&d->x.it, &c)) {
        WR(d, I2C_CMD, c);
        room--;
    }
}

static void ctrl_drain(i2c_dev_t *d)
{
    uint32_t n = fifo_lvl(d, I2C_FIFO_CRX_SHIFT);
    while (n-- > 0u) {
        rx_store(d, RD(d, I2C_CRXDATA));
    }
}

static void ctrl_drain_all(i2c_dev_t *d)
{
    uint32_t guard = 0u;
    while ((RD(d, I2C_STATUS) & I2C_STATUS_CRX_EMPTY) == 0u && guard++ < 64u) {
        rx_store(d, RD(d, I2C_CRXDATA));
    }
}

static int cstat_to_status(uint32_t cs)
{
    if (cs & I2C_CSTAT_ARB_LOST)   return I2C_ERR_ARB_LOST;
    if (cs & I2C_CSTAT_BUS_ERR)    return I2C_ERR_BUS;
    if (cs & I2C_CSTAT_SCL_TOUT)   return I2C_ERR_SCL_TIMEOUT;
    if (cs & I2C_CSTAT_NACK_ADDR)  return I2C_ERR_NACK_ADDR;
    if (cs & I2C_CSTAT_NACK_DATA)  return I2C_ERR_NACK_DATA;
    if (cs & I2C_CSTAT_CMD_ERR)    return I2C_ERR_CMD;
    if (cs & I2C_CSTAT_USER_ABORT) return I2C_ERR_ABORTED;
    return I2C_OK;
}

/*
 * After an abort the hardware flushes CMD, but a command pushed by
 * ctrl_feed() in the same instant may have survived and, if it carries
 * START, may now wait for the bus. CTRL.ABORT with the controller not
 * owning the bus flushes CMD and cancels that START without any bus
 * activity (spec 4.4.9). Then clear the status left behind.
 */
static void ctrl_cleanup(i2c_dev_t *d)
{
    ctrl_cmd(d, I2C_CTRL_ABORT);
    for (uint32_t i = 0u; i < SHORT_WAIT && (RD(d, I2C_CTRL) & I2C_CTRL_ABORT) != 0u; i++) {
    }
    WR(d, I2C_CSTAT, I2C_CSTAT_ALL);
    WR(d, I2C_INTR_STAT, SRC_XFER | I2C_INTR_CMD_STALL);
}

static void ctrl_complete(i2c_dev_t *d, int status)
{
    i2c_xfer_cb_t cb = d->x.cb;
    void *ctx = d->x.cb_ctx;
    d->x.result = status;
    d->x.busy   = false;
    if (cb != NULL) {
        cb(ctx, status);
    }
}

static void ctrl_finish(i2c_dev_t *d)
{
    mask_update(d, 0u, SRC_XFER | SRC_FEED);
    ctrl_drain_all(d);   /* ARB_LOST: these bytes belong to the lost transfer (spec 6.13) */

    int st = cstat_to_status(d->x.cstat);
    if (st == I2C_OK && d->x.rx_count != d->x.rx_expect) {
        st = I2C_ERR_HW;
    }
    if (d->x.cstat != 0u) {
        ctrl_cleanup(d);
    }
    if (st == I2C_ERR_ARB_LOST) {
        d->stats.arb_lost++;
    }
    ctrl_complete(d, st);
}

/* Controller part of the interrupt/poll service (spec 6.12.2). */
static void ctrl_service(i2c_dev_t *d, uint32_t pend)
{
    if (!d->x.busy || d->x.forced) {
        if ((d->mask & (SRC_XFER | SRC_FEED)) != 0u && !d->x.busy) {
            mask_update(d, 0u, SRC_XFER | SRC_FEED);
        }
        return;
    }

    if (pend & (SRC_XFER | I2C_INTR_SCL_TOUT | I2C_INTR_BUS_ERR)) {
        uint32_t cs = RD(d, I2C_CSTAT);
        if (cs != 0u) {
            WR(d, I2C_CSTAT, cs);
            d->x.cstat    |= cs;
            d->x.stop_feed = true;    /* CMD was flushed: do not refill */
            /* Commands pushed after the hardware flush (abort in the middle
             * of a ctrl_feed() burst) are removed while the controller is
             * still busy with the abort STOP and fetches nothing. */
            ctrl_cmd(d, I2C_CTRL_CMD_FLUSH);
        }
    }
    if (pend & (I2C_INTR_CRX_THR | I2C_INTR_CMPL)) {
        ctrl_drain(d);
    }
    if (pend & I2C_INTR_CMD_THR) {
        ctrl_feed(d);
    }
    if ((d->mask & I2C_INTR_CMD_THR) != 0u && (d->x.stop_feed || !iter_more(&d->x.it))) {
        mask_update(d, 0u, I2C_INTR_CMD_THR);
    }

    bool done = (pend & I2C_INTR_CMPL) != 0u || (d->x.cstat & CSTAT_NO_CMPL) != 0u;
    if (!done && (d->x.cstat & (I2C_CSTAT_CMD_ERR | I2C_CSTAT_USER_ABORT)) != 0u &&
        (RD(d, I2C_STATUS) & I2C_STATUS_CTRL_ACTIVE) == 0u) {
        /* abort without bus ownership: no CMPL will come (spec 4.4.5 E1) */
        done = true;
    }
    if (done) {
        ctrl_finish(d);
    }
}

int i2c_master_transfer_async(i2c_dev_t *d, const i2c_msg_t *msgs, uint32_t n,
                              i2c_xfer_cb_t cb, void *ctx)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    uint32_t rx;
    int rc = msgs_check(msgs, n, &rx);
    if (rc != I2C_OK) {
        return rc;
    }
    if ((d->ctrl & I2C_CTRL_CTRL_EN) == 0u) {
        return I2C_ERR_DISABLED;
    }
    if (d->x.busy) {
        return I2C_ERR_BUSY;
    }
    uint32_t st = RD(d, I2C_STATUS);
    if ((st & (I2C_STATUS_CTRL_ACTIVE | I2C_STATUS_BUSCLR_ACTIVE)) != 0u) {
        return I2C_ERR_BUSY;
    }
    /* Controller idle: leftovers of an earlier abort are stale (spec 4.7, 6.13). */
    if ((st & (I2C_STATUS_CMD_EMPTY | I2C_STATUS_CRX_EMPTY)) !=
        (I2C_STATUS_CMD_EMPTY | I2C_STATUS_CRX_EMPTY)) {
        ctrl_cmd(d, I2C_CTRL_CMD_FLUSH | I2C_CTRL_CRX_FLUSH);
    }
    /* Clear old events before arming them (spec 6.12.1, "enable a new source"). */
    WR(d, I2C_CSTAT, I2C_CSTAT_ALL);
    WR(d, I2C_INTR_STAT, SRC_XFER | I2C_INTR_CMD_STALL);

    uint32_t k = lock(d);
    d->x.msgs      = msgs;
    d->x.nmsgs     = n;
    iter_init(&d->x.it, msgs, n);
    d->x.rx_msg    = 0u;
    d->x.rx_idx    = 0u;
    d->x.rx_expect = rx;
    d->x.rx_count  = 0u;
    d->x.cstat     = 0u;
    d->x.stop_feed = false;
    d->x.forced    = false;
    d->x.result    = I2C_OK;
    d->x.cb        = cb;
    d->x.cb_ctx    = ctx;
    d->x.busy      = true;

    ctrl_feed(d);   /* first FIFO load; the controller starts right away */

    uint32_t arm = SRC_XFER;
    if (iter_more(&d->x.it)) {
        arm |= I2C_INTR_CMD_THR;   /* refill at CMD_LVL <= CMD_THR */
    }
    if (rx != 0u) {
        arm |= I2C_INTR_CRX_THR;   /* drain at CRX_LVL > CRX_THR   */
    }
    mask_update(d, arm, 0u);
    unlock(d, k);
    return I2C_OK;
}

bool i2c_master_busy(const i2c_dev_t *d)
{
    return d->x.busy;
}

/* Software abort / timeout: take the transfer away from the service routine,
 * abort it in hardware and end it with 'status'. */
static void ctrl_force_stop(i2c_dev_t *d, int status)
{
    uint32_t k = lock(d);
    if (!d->x.busy || d->x.forced) {
        unlock(d, k);
        return;
    }
    d->x.forced    = true;
    d->x.stop_feed = true;
    mask_update(d, 0u, SRC_XFER | SRC_FEED);
    unlock(d, k);

    ctrl_cmd(d, I2C_CTRL_ABORT);   /* abort with STOP if owning the bus */
    deadline_t dl;
    dl_start(d, &dl);
    bool stuck = false;
    while ((RD(d, I2C_CTRL) & I2C_CTRL_ABORT) != 0u ||
           (RD(d, I2C_STATUS) & I2C_STATUS_CTRL_ACTIVE) != 0u) {
        if (dl_expired(d, &dl, d->timeout)) {
            stuck = true;
            break;
        }
        idle(d);
    }
    if (stuck) {
        i2c_soft_reset(d);         /* releases SCL/SDA unconditionally */
    }
    ctrl_drain_all(d);
    WR(d, I2C_CSTAT, I2C_CSTAT_ALL);
    WR(d, I2C_INTR_STAT, SRC_XFER | I2C_INTR_CMD_STALL);
    ctrl_complete(d, status);
}

int i2c_master_abort(i2c_dev_t *d)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    ctrl_force_stop(d, I2C_ERR_ABORTED);
    return I2C_OK;
}

static int ctrl_wait(i2c_dev_t *d)
{
    deadline_t dl;
    dl_start(d, &dl);
    while (d->x.busy) {
        if (!d->use_irq) {
            i2c_poll(d);
            if (!d->x.busy) {
                break;
            }
        }
        if (dl_expired(d, &dl, d->timeout)) {
            ctrl_force_stop(d, I2C_ERR_TIMEOUT);
            break;
        }
        idle(d);
    }
    return d->x.result;
}

int i2c_master_transfer(i2c_dev_t *d, const i2c_msg_t *msgs, uint32_t n)
{
    int rc;
    uint32_t tries = 0u;
    do {
        rc = i2c_master_transfer_async(d, msgs, n, NULL, NULL);
        if (rc != I2C_OK) {
            return rc;
        }
        rc = ctrl_wait(d);
        /* After ARB_LOST the hardware waits for bus free by itself (spec 6.13). */
    } while (rc == I2C_ERR_ARB_LOST && tries++ < d->arb_retries);
    return rc;
}

/* ------------------------------------------------------------------------- */
/* Controller convenience calls                                              */
/* ------------------------------------------------------------------------- */
static uint8_t *wbuf(const uint8_t *p)
{
    return (uint8_t *)(uintptr_t)p;   /* write messages never modify buf */
}

int i2c_master_write(i2c_dev_t *d, uint8_t addr, const uint8_t *data, uint32_t len)
{
    i2c_msg_t m = { addr, 0u, len, wbuf(data) };
    return i2c_master_transfer(d, &m, 1u);
}

int i2c_master_read(i2c_dev_t *d, uint8_t addr, uint8_t *data, uint32_t len)
{
    i2c_msg_t m = { addr, I2C_M_RD, len, data };
    return i2c_master_transfer(d, &m, 1u);
}

int i2c_master_write_read(i2c_dev_t *d, uint8_t addr,
                          const uint8_t *wdata, uint32_t wlen,
                          uint8_t *rdata, uint32_t rlen)
{
    i2c_msg_t m[2] = {
        { addr, 0u,       wlen, wbuf(wdata) },
        { addr, I2C_M_RD, rlen, rdata       },
    };
    return i2c_master_transfer(d, m, 2u);
}

int i2c_reg_write(i2c_dev_t *d, uint8_t addr, const uint8_t *reg, uint32_t reg_len,
                  const uint8_t *data, uint32_t len)
{
    i2c_msg_t m[2] = {
        { addr, 0u,            reg_len, wbuf(reg)  },
        { addr, I2C_M_NOSTART, len,     wbuf(data) },
    };
    return i2c_master_transfer(d, m, (len != 0u) ? 2u : 1u);
}

int i2c_reg_read(i2c_dev_t *d, uint8_t addr, const uint8_t *reg, uint32_t reg_len,
                 uint8_t *data, uint32_t len)
{
    return i2c_master_write_read(d, addr, reg, reg_len, data, len);
}

int i2c_probe(i2c_dev_t *d, uint8_t addr)
{
    i2c_msg_t m = { addr, 0u, 0u, NULL };
    return i2c_master_transfer(d, &m, 1u);
}

int i2c_general_call_reset(i2c_dev_t *d)
{
    uint8_t b = 0x06u;
    i2c_msg_t m = { 0x00u, I2C_M_IGNORE_NAK, 1u, &b };
    return i2c_master_transfer(d, &m, 1u);
}

int i2c_read_device_id(i2c_dev_t *d, uint8_t addr, uint32_t *devid)
{
    if (devid == NULL || addr > 0x7Fu) {
        return I2C_ERR_PARAM;
    }
    uint8_t a = (uint8_t)(addr << 1);
    uint8_t id[3] = { 0u, 0u, 0u };
    i2c_msg_t m[2] = {
        { 0x7Cu, 0u,       1u, &a },   /* 0xF8 = Device ID address, W */
        { 0x7Cu, I2C_M_RD, 3u, id },   /* 0xF9 = Device ID address, R */
    };
    int rc = i2c_master_transfer(d, m, 2u);
    if (rc == I2C_OK) {
        *devid = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    }
    return rc;
}

static int ctrl_mode_bit(i2c_dev_t *d, uint32_t bit, bool en)
{
    if (d->x.busy || (RD(d, I2C_STATUS) & I2C_STATUS_CTRL_ACTIVE) != 0u) {
        return I2C_ERR_BUSY;
    }
    d->ctrl = en ? (d->ctrl | bit) : (d->ctrl & ~bit);
    WR(d, I2C_CTRL, d->ctrl);
    return I2C_OK;
}

int i2c_set_hs_mode(i2c_dev_t *d, bool enable)
{
    if (enable && !d->hs_ready) {
        return I2C_ERR_PARAM;
    }
    return ctrl_mode_bit(d, I2C_CTRL_HS_EN, enable);
}

int i2c_set_start_byte(i2c_dev_t *d, bool enable)
{
    return ctrl_mode_bit(d, I2C_CTRL_STARTBYTE_EN, enable);
}

/* ========================================================================= */
/* Target engine (spec 6.10, 6.11)                                           */
/* ========================================================================= */
static void tgt_drain(i2c_dev_t *d)
{
    const i2c_target_ops_t *ops = d->tgt_ops;
    void *ctx = d->tgt_ctx;
    uint32_t n = fifo_lvl(d, I2C_FIFO_TRX_SHIFT);

    while (n-- > 0u) {
        uint32_t e   = RD(d, I2C_TRXDATA);
        uint32_t tag = (e >> I2C_TRX_TAG_SHIFT) & I2C_TRX_TAG_MASK;
        uint8_t  b   = (uint8_t)(e & I2C_TRX_DATA_MASK);
        if (ops == NULL) {
            continue;
        }
        switch (tag) {
        case I2C_TRX_TAG_START:
        case I2C_TRX_TAG_RSTART:
            if (ops->on_start != NULL) {
                ops->on_start(ctx, (uint8_t)(b >> 1), (b & 1u) != 0u, tag == I2C_TRX_TAG_RSTART);
            }
            break;
        case I2C_TRX_TAG_DATA:
            if (ops->on_rx != NULL) {
                ops->on_rx(ctx, b);
            }
            break;
        default: /* I2C_TRX_TAG_STOP */
            if (ops->on_stop != NULL) {
                ops->on_stop(ctx);
            }
            break;
        }
    }
}

/* TX_REQ: the target stretches SCL until TTX gets data (spec 4.5.4). */
static void tgt_fill(i2c_dev_t *d)
{
    if (fifo_lvl(d, I2C_FIFO_TTX_SHIFT) != 0u) {
        return;                             /* already refilled */
    }
    uint8_t buf[32];
    size_t room = (d->fifo_depth < sizeof(buf)) ? d->fifo_depth : sizeof(buf);
    size_t n = 0u;
    if (d->tgt_ops != NULL && d->tgt_ops->on_tx != NULL) {
        n = d->tgt_ops->on_tx(d->tgt_ctx, buf, room);
    }
    if (n > room) {
        n = room;
    }
    if (n == 0u) {
        buf[0] = 0xFFu;                     /* never leave the bus stretched */
        n = 1u;
    }
    for (size_t i = 0u; i < n; i++) {
        WR(d, I2C_TTXDATA, buf[i]);
    }
}

static void tgt_service(i2c_dev_t *d, uint32_t pend)
{
    const i2c_target_ops_t *ops = d->tgt_ops;

    if (pend & I2C_INTR_TTX_DONE) {
        /* Keep TTX empty outside a read (spec 6.11 step 5). */
        uint32_t left = fifo_lvl(d, I2C_FIFO_TTX_SHIFT);
        if (left != 0u) {
            ctrl_cmd(d, I2C_CTRL_TTX_FLUSH);
        }
        if (ops != NULL && ops->on_tx_done != NULL) {
            ops->on_tx_done(d->tgt_ctx, left);
        }
    }
    if (pend & (I2C_INTR_TADDR | I2C_INTR_TSTOP | I2C_INTR_TRX_THR | I2C_INTR_TX_REQ)) {
        tgt_drain(d);                       /* always before filling TTX */
    }
    if (pend & I2C_INTR_TX_REQ) {
        tgt_fill(d);
    }
    if ((pend & I2C_INTR_GC_SWRST) != 0u && ops != NULL && ops->on_gc_swrst != NULL) {
        ops->on_gc_swrst(d->tgt_ctx);
    }
    if (pend & (I2C_INTR_TRX_OVF | I2C_INTR_TTX_UNF)) {
        if (pend & I2C_INTR_TRX_OVF) {
            d->stats.trx_ovf++;
        }
        if (pend & I2C_INTR_TTX_UNF) {
            d->stats.ttx_unf++;
        }
        if (ops != NULL && ops->on_error != NULL) {
            ops->on_error(d->tgt_ctx, pend & (I2C_INTR_TRX_OVF | I2C_INTR_TTX_UNF));
        }
    }
}

/* SCL_TOUT / BUS_ERR (spec 4.5.10, 6.13) */
static void bus_fault(i2c_dev_t *d, uint32_t pend)
{
    if (pend & I2C_INTR_BUS_ERR) {
        d->stats.bus_err++;
    }
    if (pend & I2C_INTR_SCL_TOUT) {
        d->stats.scl_tout++;
    }
    if ((d->ctrl & I2C_CTRL_TGT_EN) != 0u) {
        tgt_drain(d);                       /* deliver up to the STOP / next address */
        if ((RD(d, I2C_TSTAT) & I2C_TSTAT_RW) != 0u) {
            ctrl_cmd(d, I2C_CTRL_TTX_FLUSH); /* unknown how many bytes went out */
        }
        if (d->tgt_ops != NULL && d->tgt_ops->on_error != NULL) {
            d->tgt_ops->on_error(d->tgt_ctx, pend & (I2C_INTR_SCL_TOUT | I2C_INTR_BUS_ERR));
        }
    }
}

void i2c_target_set_ops(i2c_dev_t *d, const i2c_target_ops_t *ops, void *ctx)
{
    uint32_t k = lock(d);
    d->tgt_ops = ops;
    d->tgt_ctx = ctx;
    unlock(d, k);
}

void i2c_target_set_addr(i2c_dev_t *d, uint8_t addr0, uint8_t addr1)
{
    WR(d, I2C_TADDR, taddr_value(addr0, addr1));
}

void i2c_target_set_cfg(i2c_dev_t *d, uint32_t tcfg)
{
    WR(d, I2C_TCFG, tcfg & 0xFu);
}

/* ========================================================================= */
/* Service routine: ISR body or poll step (spec 6.12.1, 6.12.3)              */
/* ========================================================================= */
static void dma_service(i2c_dev_t *d)
{
    uint32_t s = RD(d, I2C_DMA_STAT);
    WR(d, I2C_DMA_STAT, s & I2C_DMA_STAT_W1C_ALL);
    uint32_t sel = (RD(d, I2C_DMA_CTRL) & I2C_DMA_CTRL_SEL_MASK) >> I2C_DMA_CTRL_SEL_SHIFT;
    if (d->dma_cb != NULL) {
        d->dma_cb(d->dma_ctx, sel, s);
    }
}

static void service(i2c_dev_t *d)
{
    uint32_t pend;
    /* Steps 1/2/7: read raw status, keep enabled sources, loop until none. */
    while ((pend = RD(d, I2C_INTR_STAT) & d->mask) != 0u) {
        /* Step 3: W1C the events at once, so a new one is never lost. */
        WR(d, I2C_INTR_STAT, pend & I2C_INTR_EVENT_MASK);

        ctrl_service(d, pend);
        if (pend & (I2C_INTR_SCL_TOUT | I2C_INTR_BUS_ERR)) {
            bus_fault(d, pend);
        }
        if (pend & I2C_INTR_FIFO_ERR) {
            d->stats.fifo_err++;
        }
        if (pend & I2C_INTR_DMA_DONE) {
            dma_service(d);
        }
        tgt_service(d, pend);
    }
}

void i2c_irq_handler(i2c_dev_t *d)
{
    service(d);
}

void i2c_poll(i2c_dev_t *d)
{
    if (!d->use_irq) {
        service(d);
    }
}

/* ========================================================================= */
/* Init / reset / deinit (spec 6.1, 6.2, 6.16)                               */
/* ========================================================================= */
void i2c_config_default(i2c_config_t *cfg, uint32_t pclk_hz)
{
    static const i2c_config_t zero;
    *cfg = zero;
    cfg->pclk_hz        = pclk_hz;
    cfg->speed          = I2C_SPEED_FM;
    cfg->controller     = true;
    cfg->tgt_addr[0]    = I2C_ADDR_NONE;
    cfg->tgt_addr[1]    = I2C_ADDR_NONE;
    cfg->hs_mcode       = 1u;
    cfg->scl_timeout_us = 35000u;
    cfg->arb_retries    = 3u;
}

int i2c_init(i2c_dev_t *d, uintptr_t base, const i2c_config_t *cfg)
{
    if (d == NULL || cfg == NULL || cfg->pclk_hz == 0u ||
        (unsigned)cfg->speed > (unsigned)I2C_SPEED_FMP) {
        return I2C_ERR_PARAM;
    }
    for (unsigned i = 0u; i < 2u; i++) {
        if (cfg->tgt_addr[i] != I2C_ADDR_NONE && !addr_assignable(cfg->tgt_addr[i])) {
            return I2C_ERR_PARAM;           /* reserved addresses never match */
        }
    }
    if (cfg->hs_enable && cfg->hs_scl_hz == 0u) {
        return I2C_ERR_PARAM;
    }
    if (cfg->hs_scl_hz != 0u && (cfg->hs_mcode == 0u || cfg->hs_mcode > 7u)) {
        return I2C_ERR_PARAM;               /* MCODE 000 is reserved for test */
    }

    static const i2c_dev_t zero;
    *d = zero;
    d->base        = base;
    d->plat        = cfg->plat;
    d->timeout     = cfg->timeout;
    d->arb_retries = cfg->arb_retries;
    d->use_irq     = cfg->use_irq;
    d->tgt_ops     = cfg->tgt_ops;
    d->tgt_ctx     = cfg->tgt_ctx;

    /* 6.1.1 step 4: identify the IP */
    uint32_t ver = RD(d, I2C_VERSION);
    uint32_t par = RD(d, I2C_PARAM);
    d->fifo_depth = I2C_PARAM_FIFO_DEPTH(par);
    if (I2C_VERSION_MAJOR(ver) != 2u || d->fifo_depth < 4u || d->fifo_depth > 32u) {
        return I2C_ERR_HW;
    }

    i2c_timing_t fs;
    i2c_timing_t hs = { 0 };
    int rc = i2c_calc_timing(cfg->pclk_hz, cfg->speed, cfg->scl_hz,
                             cfg->rise_ns, cfg->fall_ns, &fs);
    if (rc != I2C_OK) {
        return rc;
    }
    if (cfg->hs_scl_hz != 0u) {
        if ((par & I2C_PARAM_HS_SUP) == 0u) {
            return I2C_ERR_HW;
        }
        rc = i2c_calc_timing(cfg->pclk_hz, I2C_SPEED_HS, cfg->hs_scl_hz, 0u, 0u, &hs);
        if (rc != I2C_OK) {
            return rc;
        }
        d->hs_ready = true;
    } else {
        hs.spklen = (uint8_t)cyc_ceil(10u, cfg->pclk_hz);   /* reset-like SPKLEN_HS */
        if (hs.spklen == 0u) {
            hs.spklen = 1u;
        }
    }

    /* 6.1.2 / 6.2 step 1: clean state, IP disabled (timing registers unlock) */
    WR(d, I2C_CTRL, I2C_CTRL_SWRST);
    WR(d, I2C_CTRL, 0u);

    /* 6.2 steps 2-6b */
    WR(d, I2C_TIM_SCL, I2C_PACK16(fs.scl_high, fs.scl_low));
    WR(d, I2C_TIM_STA, I2C_PACK16(fs.tsu_sta, fs.thd_sta));
    WR(d, I2C_TIM_STO, I2C_PACK16(fs.tbuf, fs.tsu_sto));
    WR(d, I2C_TIM_DAT, I2C_PACK16(fs.tsu_dat, fs.thd_dat));
    if (d->hs_ready) {
        WR(d, I2C_HS_TIM_SCL, I2C_PACK16(hs.scl_high, hs.scl_low));
        WR(d, I2C_HS_TIM_STA, I2C_PACK16(hs.tsu_sta, hs.thd_sta));
        WR(d, I2C_HS_TIM_STO, I2C_PACK16(0u, hs.tsu_sto));
        WR(d, I2C_HS_TIM_DAT, I2C_PACK16(hs.tsu_dat, hs.thd_dat));
        WR(d, I2C_HSCFG, cfg->hs_mcode & I2C_HSCFG_MCODE_MASK);
    }
    WR(d, I2C_FILTER, ((uint32_t)hs.spklen << I2C_FILTER_HS_SHIFT) |
                      ((uint32_t)fs.spklen << I2C_FILTER_FS_SHIFT));

    uint64_t idle_ns = cfg->bus_idle_ns ? cfg->bus_idle_ns : (2u * NS_PER_S) / fs.scl_hz;
    uint32_t idle_cyc = cyc_ceil(idle_ns, cfg->pclk_hz);
    WR(d, I2C_BUSIDLE, (idle_cyc == 0u) ? 1u : (idle_cyc > 0xFFFFu ? 0xFFFFu : idle_cyc));

    if (cfg->scl_timeout_us != 0u) {
        uint64_t tc = ((uint64_t)cfg->scl_timeout_us * cfg->pclk_hz) / 1000000u;
        if (tc > I2C_TIMEOUT_VAL_MASK) {
            tc = I2C_TIMEOUT_VAL_MASK;
        }
        WR(d, I2C_TIMEOUT, I2C_TIMEOUT_EN | (uint32_t)tc);
    } else {
        WR(d, I2C_TIMEOUT, 0u);
    }

    /* 6.2 steps 7-7a: target */
    WR(d, I2C_TADDR, taddr_value(cfg->tgt_addr[0], cfg->tgt_addr[1]));
    WR(d, I2C_TCFG, cfg->tgt_cfg & 0xFu);
    if (cfg->tgt_cfg & I2C_TCFG_DEVID_EN) {
        WR(d, I2C_DEVID, cfg->devid & I2C_DEVID_MASK);
    }

    /* 6.2 step 8: thresholds. CMD refill at 1/4, CRX/TRX drain above 1/2. */
    uint32_t q = d->fifo_depth / 4u;
    uint32_t h = d->fifo_depth / 2u;
    WR(d, I2C_FIFO_CTRL, (q << I2C_FIFO_CMD_SHIFT) | (h << I2C_FIFO_CRX_SHIFT) |
                         (0u << I2C_FIFO_TTX_SHIFT) | (h << I2C_FIFO_TRX_SHIFT));

    /* 6.2 steps 8a-8b */
    WR(d, I2C_INTR_STAT, I2C_INTR_EVENT_MASK);
    WR(d, I2C_CSTAT, I2C_CSTAT_ALL);
    WR(d, I2C_DMA_STAT, I2C_DMA_STAT_W1C_ALL);
    d->mask = SRC_BASE | (cfg->target ? I2C_INTR_TGT_ALL : 0u);
    WR(d, I2C_INTR_EN, d->use_irq ? d->mask : 0u);

    /* 6.2 step 9: enable. Waiting for BUS_BUSY = 0 is not needed (step 10). */
    d->ctrl = I2C_CTRL_EN |
              (cfg->controller ? I2C_CTRL_CTRL_EN : 0u) |
              (cfg->target     ? I2C_CTRL_TGT_EN  : 0u) |
              (cfg->hs_enable  ? I2C_CTRL_HS_EN   : 0u);
    WR(d, I2C_CTRL, d->ctrl);
    return I2C_OK;
}

void i2c_soft_reset(i2c_dev_t *d)
{
    WR(d, I2C_CTRL, I2C_CTRL_SWRST);   /* FSMs, FIFOs, events cleared in 1 cycle */
    WR(d, I2C_CTRL, d->ctrl);          /* SWRST cleared EN/CTRL_EN/TGT_EN        */
}

uint32_t i2c_version(const i2c_dev_t *d)
{
    return RD(d, I2C_VERSION);
}

int i2c_deinit(i2c_dev_t *d)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    /* 6.16 step 1: stop answering new addresses */
    WR(d, I2C_TADDR, 0u);
    WR(d, I2C_TCFG, RD(d, I2C_TCFG) & ~(I2C_TCFG_GC_EN | I2C_TCFG_DEVID_EN));
    /* step 2 */
    if (d->x.busy) {
        ctrl_force_stop(d, I2C_ERR_ABORTED);
    }
    /* step 3 */
    int rc = I2C_OK;
    deadline_t dl;
    dl_start(d, &dl);
    while ((RD(d, I2C_STATUS) & (I2C_STATUS_CTRL_ACTIVE | I2C_STATUS_TGT_ACTIVE)) != 0u) {
        if (dl_expired(d, &dl, d->timeout)) {
            rc = I2C_ERR_TIMEOUT;
            break;
        }
        i2c_poll(d);
        idle(d);
    }
    /* step 4 */
    d->mask = 0u;
    WR(d, I2C_INTR_EN, 0u);
    (void)i2c_dma_stop(d);
    WR(d, I2C_DMA_CTRL, 0u);
    /* step 5 */
    d->ctrl = 0u;
    WR(d, I2C_CTRL, 0u);
    return rc;
}

/* ========================================================================= */
/* Bus recovery (spec 4.4.10, 6.14; UM 3.1.16)                               */
/* ========================================================================= */
uint32_t i2c_bus_lines(const i2c_dev_t *d)
{
    return RD(d, I2C_BUSMON) & (I2C_BUSMON_SCL_F | I2C_BUSMON_SDA_F);
}

static bool lines_free(const i2c_dev_t *d)
{
    return i2c_bus_lines(d) == (I2C_BUSMON_SCL_F | I2C_BUSMON_SDA_F);
}

int i2c_bus_clear(i2c_dev_t *d)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    if ((d->ctrl & (I2C_CTRL_EN | I2C_CTRL_CTRL_EN)) != (I2C_CTRL_EN | I2C_CTRL_CTRL_EN)) {
        return I2C_ERR_DISABLED;
    }
    if (d->x.busy) {
        return I2C_ERR_BUSY;
    }
    deadline_t dl;
    dl_start(d, &dl);
    /* step 1: controller must not own the bus */
    if ((RD(d, I2C_STATUS) & I2C_STATUS_CTRL_ACTIVE) != 0u) {
        ctrl_cmd(d, I2C_CTRL_ABORT);
        while ((RD(d, I2C_STATUS) & I2C_STATUS_CTRL_ACTIVE) != 0u) {
            if (dl_expired(d, &dl, d->timeout)) {
                return I2C_ERR_TIMEOUT;
            }
            idle(d);
        }
    }
    /* steps 3-4 */
    WR(d, I2C_INTR_STAT, I2C_INTR_BUSCLR_DONE);
    ctrl_cmd(d, I2C_CTRL_BUSCLR);
    dl_start(d, &dl);
    while ((RD(d, I2C_CTRL) & I2C_CTRL_BUSCLR) != 0u) {
        if (dl_expired(d, &dl, d->timeout)) {
            return I2C_ERR_TIMEOUT;
        }
        idle(d);
    }
    WR(d, I2C_INTR_STAT, I2C_INTR_BUSCLR_DONE);
    WR(d, I2C_CSTAT, I2C_CSTAT_ALL);
    /* step 5 */
    return lines_free(d) ? I2C_OK : I2C_ERR_BUS_STUCK;
}

int i2c_bus_clear_bitbang(i2c_dev_t *d)
{
    if (d == NULL || d->plat == NULL || d->plat->delay_us == NULL) {
        return I2C_ERR_PARAM;
    }
    if (d->x.busy) {
        return I2C_ERR_BUSY;
    }
    void (*delay)(uint32_t) = d->plat->delay_us;
    const uint32_t half = 5u;          /* 100 kHz: safe for every device on the bus */
    const uint32_t ov   = I2C_BUSMON_OVRD_EN;
    const uint32_t scl1 = I2C_BUSMON_SCL_OVRD;
    const uint32_t sda1 = I2C_BUSMON_SDA_OVRD;

    WR(d, I2C_CTRL, 0u);                /* override needs CTRL.EN = 0 (spec 4.10) */
    WR(d, I2C_BUSMON, ov | scl1 | sda1);
    delay(half);
    for (unsigned i = 0u; i < 9u; i++) {
        if ((RD(d, I2C_BUSMON) & I2C_BUSMON_SDA_F) != 0u) {
            break;                      /* the target released SDA */
        }
        WR(d, I2C_BUSMON, ov | sda1);   /* SCL low  */
        delay(half);
        WR(d, I2C_BUSMON, ov | scl1 | sda1);   /* SCL released */
        for (unsigned w = 0u; w < 1000u && (RD(d, I2C_BUSMON) & I2C_BUSMON_SCL_F) == 0u; w++) {
            delay(1u);                  /* honour clock stretching */
        }
        delay(half);
    }
    /* STOP: SCL low, SDA low, SCL high, SDA high */
    WR(d, I2C_BUSMON, ov | sda1);
    delay(half);
    WR(d, I2C_BUSMON, ov);
    delay(half);
    WR(d, I2C_BUSMON, ov | scl1);
    delay(half);
    WR(d, I2C_BUSMON, ov | scl1 | sda1);
    delay(half);
    WR(d, I2C_BUSMON, scl1 | sda1);     /* override off */

    bool ok = lines_free(d);
    WR(d, I2C_CTRL, d->ctrl);           /* re-enable: monitor waits for bus idle */
    return ok ? I2C_OK : I2C_ERR_BUS_STUCK;
}

int i2c_recover(i2c_dev_t *d)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    uint32_t l = i2c_bus_lines(d);
    if (l == (I2C_BUSMON_SCL_F | I2C_BUSMON_SDA_F)) {
        return I2C_OK;
    }
    if ((l & I2C_BUSMON_SCL_F) == 0u) {
        return I2C_ERR_BUS_STUCK;       /* SCL held: reset or power-cycle that device */
    }
    int rc = i2c_bus_clear(d);
    if (rc != I2C_OK && d->plat != NULL && d->plat->delay_us != NULL) {
        rc = i2c_bus_clear_bitbang(d);
    }
    return rc;
}

/* ========================================================================= */
/* DMA (spec 4.9, 5.2.22-5.2.24, 6.15)                                       */
/* ========================================================================= */
int i2c_dma_start(i2c_dev_t *d, i2c_dma_sel_t sel, uint32_t blk, uint32_t flags)
{
    if (d == NULL || (unsigned)sel > 3u || blk > d->fifo_depth ||
        (flags & ~(I2C_DMA_F_PFC | I2C_DMA_F_END_EN)) != 0u) {
        return I2C_ERR_PARAM;
    }
    if ((RD(d, I2C_DMA_STAT) & I2C_DMA_STAT_BUSY) != 0u ||
        (RD(d, I2C_DMA_CTRL) & I2C_DMA_CTRL_EN) != 0u) {
        return I2C_ERR_BUSY;            /* configuration is locked */
    }
    WR(d, I2C_DMA_STAT, I2C_DMA_STAT_W1C_ALL);
    WR(d, I2C_INTR_STAT, I2C_INTR_DMA_DONE);
    WR(d, I2C_DMA_BLK, blk);
    WR(d, I2C_DMA_CTRL, ((uint32_t)sel << I2C_DMA_CTRL_SEL_SHIFT) | flags | I2C_DMA_CTRL_EN);
    return I2C_OK;
}

int i2c_dma_stop(i2c_dev_t *d)
{
    if (d == NULL) {
        return I2C_ERR_PARAM;
    }
    uint32_t keep = RD(d, I2C_DMA_CTRL) &
                    (I2C_DMA_CTRL_SEL_MASK | I2C_DMA_CTRL_PFC | I2C_DMA_CTRL_END_EN);
    WR(d, I2C_DMA_CTRL, keep | I2C_DMA_CTRL_CLR);   /* EN = 0, withdraw request */
    deadline_t dl;
    dl_start(d, &dl);
    while ((RD(d, I2C_DMA_STAT) & I2C_DMA_STAT_BUSY) != 0u) {
        if (dl_expired(d, &dl, d->timeout)) {
            return I2C_ERR_TIMEOUT;     /* ack in flight: the DMAC must finish it */
        }
        idle(d);
    }
    return I2C_OK;
}

void i2c_dma_set_callback(i2c_dev_t *d, i2c_dma_cb_t cb, void *ctx)
{
    uint32_t k = lock(d);
    d->dma_cb  = cb;
    d->dma_ctx = ctx;
    unlock(d, k);
    if (cb != NULL) {
        mask_update(d, I2C_INTR_DMA_DONE, 0u);
    } else {
        mask_update(d, 0u, I2C_INTR_DMA_DONE);
    }
}
