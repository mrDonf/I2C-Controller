/*
 * apb_i2c_regs.h — Register map of the apb_i2c IP (architecture spec v1.5, IP v2.1.0)
 *
 * Offsets, bit fields and command encodings exactly as in spec section 5.
 * This file has no dependency other than <stdint.h>; it can be used alone
 * (e.g. by a boot ROM) without the rest of the driver.
 */
#ifndef APB_I2C_REGS_H
#define APB_I2C_REGS_H

#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Register offsets (spec 5.1)                                               */
/* ------------------------------------------------------------------------- */
#define I2C_CTRL            0x000u  /* RW/W1SC */
#define I2C_STATUS          0x004u  /* RO      */
#define I2C_INTR_STAT       0x008u  /* RO/W1C  */
#define I2C_INTR_EN         0x00Cu  /* RW      */
#define I2C_INTR_TEST       0x010u  /* WO      */
#define I2C_FIFO_CTRL       0x014u  /* RW      */
#define I2C_FIFO_STAT       0x018u  /* RO      */
#define I2C_CMD             0x01Cu  /* WO, push CMD FIFO  */
#define I2C_CRXDATA         0x020u  /* RO, pop CRX FIFO   */
#define I2C_TTXDATA         0x024u  /* WO, push TTX FIFO  */
#define I2C_TRXDATA         0x028u  /* RO, pop TRX FIFO   */
#define I2C_TADDR           0x02Cu
#define I2C_TCFG            0x030u
#define I2C_HSCFG           0x034u
#define I2C_CSTAT           0x038u  /* RO/W1C  */
#define I2C_TSTAT           0x03Cu  /* RO      */
#define I2C_TIM_SCL         0x040u  /* RW, locked while CTRL.EN = 1 */
#define I2C_TIM_STA         0x044u
#define I2C_TIM_STO         0x048u
#define I2C_TIM_DAT         0x04Cu
#define I2C_HS_TIM_SCL      0x050u
#define I2C_HS_TIM_STA      0x054u
#define I2C_HS_TIM_STO      0x058u
#define I2C_HS_TIM_DAT      0x05Cu
#define I2C_FILTER          0x060u  /* RW, locked */
#define I2C_TIMEOUT         0x064u
#define I2C_BUSIDLE         0x068u  /* RW, locked */
#define I2C_DEVID           0x06Cu
#define I2C_BUSMON          0x070u
#define I2C_DMA_CTRL        0x074u
#define I2C_DMA_BLK         0x078u
#define I2C_DMA_STAT        0x07Cu  /* RO/W1C  */
#define I2C_PARAM           0x0F8u
#define I2C_VERSION         0x0FCu

/* ------------------------------------------------------------------------- */
/* CTRL (spec 5.2.1)                                                         */
/* ------------------------------------------------------------------------- */
#define I2C_CTRL_EN             (1u << 0)
#define I2C_CTRL_CTRL_EN        (1u << 1)
#define I2C_CTRL_TGT_EN         (1u << 2)
#define I2C_CTRL_HS_EN          (1u << 3)
#define I2C_CTRL_STARTBYTE_EN   (1u << 4)
#define I2C_CTRL_CMD_FLUSH      (1u << 8)   /* W1SC */
#define I2C_CTRL_CRX_FLUSH      (1u << 9)   /* W1SC */
#define I2C_CTRL_TTX_FLUSH      (1u << 10)  /* W1SC */
#define I2C_CTRL_TRX_FLUSH      (1u << 11)  /* W1SC */
#define I2C_CTRL_ABORT          (1u << 16)  /* W1SC, reads 1 while aborting  */
#define I2C_CTRL_BUSCLR         (1u << 17)  /* W1SC, reads 1 while clearing  */
#define I2C_CTRL_SWRST          (1u << 31)  /* W1SC, reads 0 */
/* Plain RW bits: preserve them, and only them, in read-modify-write */
#define I2C_CTRL_RW_MASK        0x0000001Fu

/* ------------------------------------------------------------------------- */
/* STATUS (spec 5.2.2)                                                       */
/* ------------------------------------------------------------------------- */
#define I2C_STATUS_BUS_BUSY       (1u << 0)
#define I2C_STATUS_CTRL_ACTIVE    (1u << 1)
#define I2C_STATUS_CTRL_HOLD      (1u << 2)
#define I2C_STATUS_TGT_ACTIVE     (1u << 3)
#define I2C_STATUS_TGT_RW         (1u << 4)
#define I2C_STATUS_TGT_HOLD       (1u << 5)
#define I2C_STATUS_HS_ACTIVE      (1u << 6)
#define I2C_STATUS_BUSCLR_ACTIVE  (1u << 7)
#define I2C_STATUS_CMD_FULL       (1u << 8)
#define I2C_STATUS_CMD_EMPTY      (1u << 9)
#define I2C_STATUS_CRX_FULL       (1u << 10)
#define I2C_STATUS_CRX_EMPTY      (1u << 11)
#define I2C_STATUS_TTX_FULL       (1u << 12)
#define I2C_STATUS_TTX_EMPTY      (1u << 13)
#define I2C_STATUS_TRX_FULL       (1u << 14)
#define I2C_STATUS_TRX_EMPTY      (1u << 15)

/* ------------------------------------------------------------------------- */
/* INTR_STAT / INTR_EN / INTR_TEST (spec 4.8, 5.2.3)                         */
/* ------------------------------------------------------------------------- */
#define I2C_INTR_CMD_THR      (1u << 0)   /* level */
#define I2C_INTR_CRX_THR      (1u << 1)   /* level */
#define I2C_INTR_CMPL         (1u << 2)
#define I2C_INTR_NACK         (1u << 3)
#define I2C_INTR_ARB_LOST     (1u << 4)
#define I2C_INTR_CMD_ERR      (1u << 5)
#define I2C_INTR_CMD_STALL    (1u << 6)
#define I2C_INTR_BUSCLR_DONE  (1u << 7)
#define I2C_INTR_TRX_THR      (1u << 8)   /* level */
#define I2C_INTR_TTX_THR      (1u << 9)   /* level */
#define I2C_INTR_TX_REQ       (1u << 10)
#define I2C_INTR_TADDR        (1u << 11)
#define I2C_INTR_TSTOP        (1u << 12)
#define I2C_INTR_TTX_DONE     (1u << 13)
#define I2C_INTR_GC_SWRST     (1u << 14)
#define I2C_INTR_HS_DET       (1u << 15)
#define I2C_INTR_TRX_OVF      (1u << 16)
#define I2C_INTR_TTX_UNF      (1u << 17)
#define I2C_INTR_SCL_TOUT     (1u << 18)
#define I2C_INTR_BUS_ERR      (1u << 19)
#define I2C_INTR_FIFO_ERR     (1u << 20)
#define I2C_INTR_DMA_DONE     (1u << 21)
#define I2C_INTR_EVENT_MASK   0x003FFCFCu  /* every W1C event bit */
#define I2C_INTR_LEVEL_MASK   0x00000303u  /* CMD_THR, CRX_THR, TRX_THR, TTX_THR */

/* Controller events that end, or report on, one transfer */
#define I2C_INTR_CTRL_XFER    (I2C_INTR_CMPL | I2C_INTR_NACK | I2C_INTR_ARB_LOST | \
                               I2C_INTR_CMD_ERR | I2C_INTR_CMD_STALL)
/* Recommended target set (spec 6.12) */
#define I2C_INTR_TGT_ALL      (I2C_INTR_TRX_THR | I2C_INTR_TX_REQ | I2C_INTR_TADDR | \
                               I2C_INTR_TSTOP | I2C_INTR_TTX_DONE | I2C_INTR_GC_SWRST | \
                               I2C_INTR_TRX_OVF | I2C_INTR_TTX_UNF)
#define I2C_INTR_ERR_BUS      (I2C_INTR_SCL_TOUT | I2C_INTR_BUS_ERR)

/* ------------------------------------------------------------------------- */
/* FIFO_CTRL / FIFO_STAT (spec 5.2.4, 5.2.5)                                 */
/* ------------------------------------------------------------------------- */
#define I2C_FIFO_CMD_SHIFT    0
#define I2C_FIFO_CRX_SHIFT    8
#define I2C_FIFO_TTX_SHIFT    16
#define I2C_FIFO_TRX_SHIFT    24
#define I2C_FIFO_FIELD_MASK   0x3Fu
#define I2C_FIFO_LVL(stat, shift)  (((stat) >> (shift)) & I2C_FIFO_FIELD_MASK)

/* ------------------------------------------------------------------------- */
/* CMD (spec 4.4.1, 5.2.6) — one command = one byte on the bus               */
/* ------------------------------------------------------------------------- */
#define I2C_CMD_DATA_MASK     0x000000FFu
#define I2C_CMD_START         (1u << 8)   /* S / Sr before this (address) byte */
#define I2C_CMD_STOP          (1u << 9)   /* P after this byte                 */
#define I2C_CMD_READ          (1u << 10)  /* receive one byte                  */
#define I2C_CMD_NACK          (1u << 11)  /* NACK the received byte (last one) */
#define I2C_CMD_NAKOK         (1u << 12)  /* do not abort on NACK when sending */

#define I2C_CMD_ADDR_W(a7)    (I2C_CMD_START | (((uint32_t)(a7) & 0x7Fu) << 1))
#define I2C_CMD_ADDR_R(a7)    (I2C_CMD_START | (((uint32_t)(a7) & 0x7Fu) << 1) | 1u)

/* ------------------------------------------------------------------------- */
/* CRXDATA / TRXDATA (spec 4.5.2, 5.2.7, 5.2.9)                              */
/* ------------------------------------------------------------------------- */
#define I2C_CRX_DATA_MASK     0xFFu
#define I2C_CRX_LAST          (1u << 8)

#define I2C_TRX_DATA_MASK     0xFFu
#define I2C_TRX_TAG_SHIFT     8
#define I2C_TRX_TAG_MASK      0x3u
#define I2C_TRX_TAG_DATA      0u   /* data byte                       */
#define I2C_TRX_TAG_START     1u   /* S  + address byte               */
#define I2C_TRX_TAG_STOP      2u   /* P (DATA = 0)                    */
#define I2C_TRX_TAG_RSTART    3u   /* Sr + address byte               */

/* ------------------------------------------------------------------------- */
/* TADDR / TCFG / HSCFG (spec 5.2.10 – 5.2.12)                               */
/* ------------------------------------------------------------------------- */
#define I2C_TADDR_ADDR0_SHIFT 0
#define I2C_TADDR_ADDR0_EN    (1u << 7)
#define I2C_TADDR_ADDR1_SHIFT 8
#define I2C_TADDR_ADDR1_EN    (1u << 15)

#define I2C_TCFG_GC_EN        (1u << 0)
#define I2C_TCFG_NOSTRETCH    (1u << 1)
#define I2C_TCFG_TNACK        (1u << 2)
#define I2C_TCFG_DEVID_EN     (1u << 3)

#define I2C_HSCFG_MCODE_MASK  0x7u

/* ------------------------------------------------------------------------- */
/* CSTAT — controller abort cause (spec 5.2.13), W1C                         */
/* ------------------------------------------------------------------------- */
#define I2C_CSTAT_NACK_ADDR   (1u << 0)
#define I2C_CSTAT_NACK_DATA   (1u << 1)
#define I2C_CSTAT_ARB_LOST    (1u << 2)
#define I2C_CSTAT_USER_ABORT  (1u << 3)
#define I2C_CSTAT_CMD_ERR     (1u << 4)
#define I2C_CSTAT_SCL_TOUT    (1u << 5)
#define I2C_CSTAT_BUS_ERR     (1u << 6)
#define I2C_CSTAT_ALL         0x7Fu

/* ------------------------------------------------------------------------- */
/* TSTAT (spec 5.2.14)                                                       */
/* ------------------------------------------------------------------------- */
#define I2C_TSTAT_ADDR_MASK     0x7Fu
#define I2C_TSTAT_RW            (1u << 7)
#define I2C_TSTAT_GC            (1u << 8)
#define I2C_TSTAT_ADDR1_HIT     (1u << 9)
#define I2C_TSTAT_DEVID_ACTIVE  (1u << 10)

/* ------------------------------------------------------------------------- */
/* Timing / filter / timeout / idle (spec 5.2.15 – 5.2.19)                   */
/* Every timing register packs two 16-bit fields: {[31:16], [15:0]}          */
/* ------------------------------------------------------------------------- */
#define I2C_PACK16(hi, lo)    ((((uint32_t)(hi) & 0xFFFFu) << 16) | ((uint32_t)(lo) & 0xFFFFu))
/* TIM_SCL = {SCL_HIGH, SCL_LOW}   TIM_STA = {TSU_STA, THD_STA}
 * TIM_STO = {TBUF, TSU_STO}       TIM_DAT = {TSU_DAT, THD_DAT}              */
#define I2C_FILTER_FS_SHIFT   0
#define I2C_FILTER_HS_SHIFT   8
#define I2C_TIMEOUT_VAL_MASK  0x00FFFFFFu
#define I2C_TIMEOUT_EN        (1u << 31)
#define I2C_BUSIDLE_MASK      0xFFFFu

/* ------------------------------------------------------------------------- */
/* DEVID (spec 5.2.20)                                                       */
/* ------------------------------------------------------------------------- */
#define I2C_DEVID_MASK        0x00FFFFFFu
#define I2C_DEVID_MANUF(id)   (((id) >> 12) & 0xFFFu)
#define I2C_DEVID_PART(id)    (((id) >> 3) & 0x1FFu)
#define I2C_DEVID_REV(id)     ((id) & 0x7u)

/* ------------------------------------------------------------------------- */
/* BUSMON (spec 5.2.21)                                                      */
/* ------------------------------------------------------------------------- */
#define I2C_BUSMON_SCL_F      (1u << 0)
#define I2C_BUSMON_SDA_F      (1u << 1)
#define I2C_BUSMON_SCL_OE     (1u << 2)
#define I2C_BUSMON_SDA_OE     (1u << 3)
#define I2C_BUSMON_OVRD_EN    (1u << 8)   /* effective only with CTRL.EN = 0 */
#define I2C_BUSMON_SCL_OVRD   (1u << 9)   /* 1 = release, 0 = pull low */
#define I2C_BUSMON_SDA_OVRD   (1u << 10)

/* ------------------------------------------------------------------------- */
/* DMA_CTRL / DMA_BLK / DMA_STAT (spec 5.2.22 – 5.2.24)                      */
/* ------------------------------------------------------------------------- */
#define I2C_DMA_CTRL_EN       (1u << 0)
#define I2C_DMA_CTRL_PFC      (1u << 1)
#define I2C_DMA_CTRL_END_EN   (1u << 2)
#define I2C_DMA_CTRL_CLR      (1u << 3)
#define I2C_DMA_CTRL_SEL_SHIFT 4
#define I2C_DMA_CTRL_SEL_MASK (3u << 4)

#define I2C_DMA_STAT_REQ      (1u << 0)
#define I2C_DMA_STAT_BUSY     (1u << 1)
#define I2C_DMA_STAT_DONE     (1u << 8)
#define I2C_DMA_STAT_DENY     (1u << 16)
#define I2C_DMA_STAT_END      (1u << 24)
#define I2C_DMA_STAT_W1C_ALL  (I2C_DMA_STAT_DONE | I2C_DMA_STAT_DENY | I2C_DMA_STAT_END)

/* ------------------------------------------------------------------------- */
/* PARAM / VERSION (spec 5.2.25)                                             */
/* ------------------------------------------------------------------------- */
#define I2C_PARAM_FIFO_DEPTH(p)  ((p) & 0xFFu)
#define I2C_PARAM_HS_SUP         (1u << 8)
#define I2C_PARAM_DEVID_SUP      (1u << 9)
#define I2C_PARAM_DMA_SUP        (1u << 10)
#define I2C_PARAM_DMA_END_SUP    (1u << 11)
#define I2C_VERSION_MAJOR(v)     (((v) >> 16) & 0xFFFFu)
#define I2C_VERSION_MINOR(v)     (((v) >> 8) & 0xFFu)

#endif /* APB_I2C_REGS_H */
