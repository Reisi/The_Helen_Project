/**
  ******************************************************************************
  * @file    drake_regs.h
  * @author  Thomas Reisnecker
  * @brief   register definitions for drake i2c interface
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRAKE_REGS_H_INCLUDED
#define DRAKE_REGS_H_INCLUDED

/* device address ------------------------------------------------------------*/
#define DRK_ADDR               0x30

/* device registers ----------------------------------------------------------*/
#define DRK_REG_PWR            0x00
#define DRK_REG_STATUS0        0x01
#define DRK_REG_STATUS1        0x02
#define DRK_REG_MDATA          0x03
#define DRK_REG_TDATA          0x04
#define DRK_REG_WHOAMI         0x05
#define DRK_REG_MTCFG0         0x06
#define DRK_REG_MTCFG1         0x07
#define DRK_REG_MTCFG2         0x08
#define DRK_REG_MTCFG3         0x09
#define DRK_REG_MTCAL0         0x0A
#define DRK_REG_MTCAL1         0x0B
#define DRK_REG_MTCAL2         0x0C
#define DRK_REG_MTCAL3         0x0D
#define DRK_REG_MTCAL4         0x0E
#define DRK_REG_TRVCAL0        0x0F
#define DRK_REG_TRVCAL1        0x10
#define DRK_REG_TRVCAL2        0x11
#define DRK_REG_TRVCAL3        0x12
#define DRK_REG_TRVCAL4        0x13
#define DRK_REG_INDCFG0        0x14
#define DRK_REG_INDCFG1        0x15
#define DRK_REG_INDPOS0        0x16
#define DRK_REG_INDPOS1        0x17
#define DRK_REG_INDPOS2        0x18
#define DRK_REG_INDPOS3        0x19

#define DRK_REG_CNT            26

#define DRK_NUM_OF_IND_POS     4

/* PWR register -------------------------------------------------------------*/
#define DRK_PWR_SYS_OFF_MASK   (1 << 0)

/* STATUS registers ---------------------------------------------------------*/
#define DRK_STATUS_MPR_MASK    (1 << 0)
#define DRK_STATUS_TPR_MASK    (1 << 1)
#define DRK_STATUS_MDR_MASK    (1 << 0)
#define DRK_STATUS_TDR_MASK    (1 << 1)
#define DRK_STATUS_MOR_MASK    (1 << 4)
#define DRK_STATUS_TOR_MASK    (1 << 5)

/* CALIB registers ----------------------------------------------------------*/
#define DRK_CALIB_THC_MASK     (1 << 0)
#define DRK_CALIB_FOC_MASK     (1 << 1)
#define DRK_CALIB_THS_MASK     (1 << 4)
#define DRK_CALIB_FOS_MASK     (1 << 5)
#define DRK_CALIB_TOC_MASK     (1 << 0)

/* WHOAMI register ----------------------------------------------------------*/
#define DRK_WHO_AM_I           0xDE

#endif // DRAKE_REGS_H_INCLUDED

/**END OF FILE*****************************************************************/

