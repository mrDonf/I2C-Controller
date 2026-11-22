/*
 * example_main.c — typical use of the apb_i2c bare-metal driver
 *
 * Board assumptions (adapt to the SoC):
 *   I2C0 at 0x40010000, PCLK 100 MHz, irq_o on interrupt line I2C0_IRQn
 *   SysTick-like microsecond counter in us_counter()
 *
 * I2C0 is controller + target at the same time:
 *   - controller: reads a temperature sensor (address 0x48, 16-bit register 0x00)
 *     and writes 32 bytes into an EEPROM (address 0x50, 2-byte memory address)
 *   - target (address 0x3A): exposes a 64-byte register file to another host
 *     ("S 0x74 reg ..." writes, "S 0x74 reg Sr 0x75 ..." reads)
 */
#include <stdint.h>

#include "apb_i2c.h"

#define I2C0_BASE   0x40010000u
#define PCLK_HZ     100000000u

/* ------------------------------------------------------------------------- */
/* Platform (replace with the real ones)                                     */
/* ------------------------------------------------------------------------- */
extern uint32_t us_counter(void);
extern void     cpu_wfi(void);
extern uint32_t irq_disable(void);          /* returns previous state */
extern void     irq_restore(uint32_t key);

static void wait_irq(void *arg)
{
    (void)arg;
    cpu_wfi();                               /* sleep until the next interrupt */
}

static void delay_us(uint32_t us)
{
    uint32_t t0 = us_counter();
    while ((uint32_t)(us_counter() - t0) < us) {
    }
}

static const i2c_platform_t plat = {
    us_counter, delay_us, wait_irq, NULL, irq_disable, irq_restore
};

static i2c_dev_t i2c0;

/* Interrupt vector entry */
void I2C0_IRQHandler(void)
{
    i2c_irq_handler(&i2c0);
}

/* ------------------------------------------------------------------------- */
/* Target: 64-byte register file                                             */
/* ------------------------------------------------------------------------- */
static struct {
    uint8_t regs[64];
    uint8_t ptr;
    bool    want_ptr;
} rf;

static void on_start(void *ctx, uint8_t addr7, bool read, bool repeated)
{
    (void)ctx; (void)addr7; (void)repeated;
    rf.want_ptr = !read;                     /* first written byte = register index */
}

static void on_rx(void *ctx, uint8_t b)
{
    (void)ctx;
    if (rf.want_ptr) {
        rf.ptr = (uint8_t)(b & 63u);
        rf.want_ptr = false;
    } else {
        rf.regs[rf.ptr] = b;
        rf.ptr = (uint8_t)((rf.ptr + 1u) & 63u);
    }
}

static size_t on_tx(void *ctx, uint8_t *buf, size_t max)
{
    (void)ctx;
    for (size_t i = 0; i < max; i++) {
        buf[i] = rf.regs[rf.ptr];
        rf.ptr = (uint8_t)((rf.ptr + 1u) & 63u);
    }
    return max;
}

static void on_tx_done(void *ctx, uint32_t unsent)
{
    (void)ctx;
    rf.ptr = (uint8_t)((rf.ptr - unsent) & 63u);   /* give back prefetched bytes */
}

static const i2c_target_ops_t tgt_ops = {
    on_start, on_rx, NULL, on_tx, on_tx_done, NULL, NULL
};

/* ------------------------------------------------------------------------- */
/* Controller helpers                                                        */
/* ------------------------------------------------------------------------- */
static int read_temperature(int16_t *centi_deg)
{
    uint8_t reg = 0x00u, raw[2];
    int rc = i2c_reg_read(&i2c0, 0x48u, &reg, 1u, raw, 2u);
    if (rc == I2C_OK) {
        int16_t v = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
        *centi_deg = (int16_t)((v >> 7) * 50);      /* LM75 style, 0.5 degC/LSB */
    }
    return rc;
}

static int eeprom_write_page(uint16_t mem, const uint8_t *data, uint32_t len)
{
    uint8_t a[2] = { (uint8_t)(mem >> 8), (uint8_t)mem };
    int rc = i2c_reg_write(&i2c0, 0x50u, a, 2u, data, len);
    if (rc != I2C_OK) {
        return rc;
    }
    /* acknowledge polling: the EEPROM NACKs its address while it programs */
    for (int i = 0; i < 100; i++) {
        rc = i2c_probe(&i2c0, 0x50u);
        if (rc != I2C_ERR_NACK_ADDR) {
            break;
        }
        delay_us(100u);
    }
    return rc;
}

static void handle_error(int rc)
{
    switch (rc) {
    case I2C_ERR_SCL_TIMEOUT:
    case I2C_ERR_BUS:
    case I2C_ERR_TIMEOUT:
        /* SDA stuck low -> bus clear; SCL stuck low -> reset that device */
        if (i2c_recover(&i2c0) != I2C_OK) {
            i2c_soft_reset(&i2c0);
        }
        break;
    default:
        break;                               /* NACK: no device / busy device */
    }
}

int main(void)
{
    i2c_config_t cfg;
    i2c_config_default(&cfg, PCLK_HZ);
    cfg.speed          = I2C_SPEED_FM;       /* 400 kHz */
    cfg.rise_ns        = 120u;               /* measured on the board */
    cfg.fall_ns        = 20u;
    cfg.controller     = true;
    cfg.target         = true;
    cfg.tgt_addr[0]    = 0x3Au;
    cfg.tgt_cfg        = I2C_TCFG_GC_EN;
    cfg.tgt_ops        = &tgt_ops;
    cfg.scl_timeout_us = 35000u;             /* SMBus-like */
    cfg.timeout        = 100000u;            /* 100 ms per blocking call */
    cfg.use_irq        = true;
    cfg.plat           = &plat;

    if (i2c_init(&i2c0, I2C0_BASE, &cfg) != I2C_OK) {
        for (;;) {
        }
    }
    /* enable I2C0_IRQn in the interrupt controller here */

    static const uint8_t page[32] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    int rc = eeprom_write_page(0x0100u, page, sizeof(page));
    if (rc != I2C_OK) {
        handle_error(rc);
    }

    for (;;) {
        int16_t t;
        rc = read_temperature(&t);
        if (rc == I2C_OK) {
            rf.regs[0] = (uint8_t)((uint16_t)t >> 8);   /* publish on the target side */
            rf.regs[1] = (uint8_t)t;
        } else {
            handle_error(rc);
        }
        delay_us(100000u);
    }
}
