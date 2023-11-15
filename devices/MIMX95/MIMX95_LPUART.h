/*
** ###################################################################
**     Processors:          MIMX95_cm33
**
**     Compilers:           GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**
**     Reference manual:    IMX95RM, Internal, November. 2021
**     Version:             rev. 1.0, 2021-11-16
**     Build:               b220818
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MIMX95_cm33
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2022 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2021-11-16)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MIMX95_LPUART.h
 * @version 1.0
 * @date 2021-11-16
 * @brief CMSIS Peripheral Access Layer for MIMX95_LPUART
 *
 * CMSIS Peripheral Access Layer for MIMX95
 */

/* Prevention from multiple including the same memory map */
#if !defined(MIMX95_LPUART_H_)  /* Check if memory map has not been already included */
#define MIMX95_LPUART_H_

#include "MIMX95_COMMON.h"

/* ----------------------------------------------------------------------------
   -- LPUART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Peripheral_Access_Layer LPUART Peripheral Access Layer
 * @{
 */

/** LPUART - Register Layout Typedef */
typedef struct
{
    __I uint32_t VERID;   /**< Version ID Register, offset: 0x0 */
    __I uint32_t PARAM;   /**< Parameter Register, offset: 0x4 */
    __IO uint32_t GLOBAL; /**< LPUART Global Register, offset: 0x8 */
    __IO uint32_t PINCFG; /**< LPUART Pin Configuration Register, offset: 0xC */
    __IO uint32_t BAUD;   /**< LPUART Baud Rate Register, offset: 0x10 */
    __IO uint32_t STAT;   /**< LPUART Status Register, offset: 0x14 */
    __IO uint32_t CTRL;   /**< LPUART Control Register, offset: 0x18 */
    __IO uint32_t DATA;   /**< LPUART Data Register, offset: 0x1C */
    __IO uint32_t MATCH;  /**< LPUART Match Address Register, offset: 0x20 */
    __IO uint32_t MODIR;  /**< LPUART Modem IrDA Register, offset: 0x24 */
    __IO uint32_t FIFO;   /**< LPUART FIFO Register, offset: 0x28 */
    __IO uint32_t WATER;  /**< LPUART Watermark Register, offset: 0x2C */
    __I uint32_t DATARO;  /**< Data read-only Register, offset: 0x30 */
    uint8_t RESERVED_0[12];
    __IO uint32_t MCR;  /**< MODEM Control Register, offset: 0x40 */
    __IO uint32_t MSR;  /**< MODEM Status Register, offset: 0x44 */
    __IO uint32_t REIR; /**< Receiver Extended Idle Register, offset: 0x48 */
    __IO uint32_t TEIR; /**< Transmitter Extended Idle Register, offset: 0x4C */
    __IO uint32_t HDCR; /**< Half Duplex Control Register, offset: 0x50 */
    uint8_t RESERVED_1[4];
    __IO uint32_t TOCR;       /**< Timeout Control Register, offset: 0x58 */
    __IO uint32_t TOSR;       /**< Timeout Status Register, offset: 0x5C */
    __IO uint32_t TIMEOUT[4]; /**< Timeout N Register, array offset: 0x60, array step: 0x4 */
    uint8_t RESERVED_2[400];
    __O uint32_t TCBR[128]; /**< Transmit Command Burst Register, array offset: 0x200, array step: 0x4 */
    __O uint32_t TDBR[256]; /**< Transmit Data Burst Register, array offset: 0x400, array step: 0x4 */
} LPUART_Type;

/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/*! @name VERID - Version ID Register */
/*! @{ */

#define LPUART_VERID_FEATURE_MASK  (0xFFFFU)
#define LPUART_VERID_FEATURE_SHIFT (0U)
/*! FEATURE - Feature Identification Number
 *  0b0000000000000001..Standard feature set.
 *  0b0000000000000011..Standard feature set with MODEM/IrDA support.
 *  0b0000000000000111..Enhanced feature set with full MODEM, IrDA and enhanced idle detection.
 */
#define LPUART_VERID_FEATURE(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_FEATURE_SHIFT)) & LPUART_VERID_FEATURE_MASK)

#define LPUART_VERID_MINOR_MASK  (0xFF0000U)
#define LPUART_VERID_MINOR_SHIFT (16U)
/*! MINOR - Minor Version Number
 */
#define LPUART_VERID_MINOR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MINOR_SHIFT)) & LPUART_VERID_MINOR_MASK)

#define LPUART_VERID_MAJOR_MASK  (0xFF000000U)
#define LPUART_VERID_MAJOR_SHIFT (24U)
/*! MAJOR - Major Version Number
 */
#define LPUART_VERID_MAJOR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_VERID_MAJOR_SHIFT)) & LPUART_VERID_MAJOR_MASK)
/*! @} */

/*! @name PARAM - Parameter Register */
/*! @{ */

#define LPUART_PARAM_TXFIFO_MASK  (0xFFU)
#define LPUART_PARAM_TXFIFO_SHIFT (0U)
/*! TXFIFO - Transmit FIFO Size
 */
#define LPUART_PARAM_TXFIFO(x) (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_TXFIFO_SHIFT)) & LPUART_PARAM_TXFIFO_MASK)

#define LPUART_PARAM_RXFIFO_MASK  (0xFF00U)
#define LPUART_PARAM_RXFIFO_SHIFT (8U)
/*! RXFIFO - Receive FIFO Size
 */
#define LPUART_PARAM_RXFIFO(x) (((uint32_t)(((uint32_t)(x)) << LPUART_PARAM_RXFIFO_SHIFT)) & LPUART_PARAM_RXFIFO_MASK)
/*! @} */

/*! @name GLOBAL - LPUART Global Register */
/*! @{ */

#define LPUART_GLOBAL_RST_MASK  (0x2U)
#define LPUART_GLOBAL_RST_SHIFT (1U)
/*! RST - Software Reset
 *  0b0..Module is not reset.
 *  0b1..Module is reset.
 */
#define LPUART_GLOBAL_RST(x) (((uint32_t)(((uint32_t)(x)) << LPUART_GLOBAL_RST_SHIFT)) & LPUART_GLOBAL_RST_MASK)
/*! @} */

/*! @name PINCFG - LPUART Pin Configuration Register */
/*! @{ */

#define LPUART_PINCFG_TRGSEL_MASK  (0x3U)
#define LPUART_PINCFG_TRGSEL_SHIFT (0U)
/*! TRGSEL - Trigger Select
 *  0b00..Input trigger is disabled.
 *  0b01..Input trigger is used instead of RXD pin input.
 *  0b10..Input trigger is used instead of CTS_B pin input.
 *  0b11..Input trigger is used to modulate the TXD pin output. The TXD pin output (after TXINV configuration) is
 *        internally ANDed with the input trigger.
 */
#define LPUART_PINCFG_TRGSEL(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_PINCFG_TRGSEL_SHIFT)) & LPUART_PINCFG_TRGSEL_MASK)
/*! @} */

/*! @name BAUD - LPUART Baud Rate Register */
/*! @{ */

#define LPUART_BAUD_SBR_MASK  (0x1FFFU)
#define LPUART_BAUD_SBR_SHIFT (0U)
/*! SBR - Baud Rate Modulo Divisor.
 */
#define LPUART_BAUD_SBR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBR_SHIFT)) & LPUART_BAUD_SBR_MASK)

#define LPUART_BAUD_SBNS_MASK  (0x2000U)
#define LPUART_BAUD_SBNS_SHIFT (13U)
/*! SBNS - Stop Bit Number Select
 *  0b0..One stop bit.
 *  0b1..Two stop bits.
 */
#define LPUART_BAUD_SBNS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_SBNS_SHIFT)) & LPUART_BAUD_SBNS_MASK)

#define LPUART_BAUD_RXEDGIE_MASK  (0x4000U)
#define LPUART_BAUD_RXEDGIE_SHIFT (14U)
/*! RXEDGIE - RX Input Active Edge Interrupt Enable
 *  0b0..Hardware interrupts from STAT[RXEDGIF] are disabled.
 *  0b1..Hardware interrupt is requested when STAT[RXEDGIF] flag is 1.
 */
#define LPUART_BAUD_RXEDGIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RXEDGIE_SHIFT)) & LPUART_BAUD_RXEDGIE_MASK)

#define LPUART_BAUD_LBKDIE_MASK  (0x8000U)
#define LPUART_BAUD_LBKDIE_SHIFT (15U)
/*! LBKDIE - LIN Break Detect Interrupt Enable
 *  0b0..Hardware interrupts from STAT[LBKDIF] flag are disabled (use polling).
 *  0b1..Hardware interrupt is requested when STAT[LBKDIF] flag is 1.
 */
#define LPUART_BAUD_LBKDIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_LBKDIE_SHIFT)) & LPUART_BAUD_LBKDIE_MASK)

#define LPUART_BAUD_RESYNCDIS_MASK  (0x10000U)
#define LPUART_BAUD_RESYNCDIS_SHIFT (16U)
/*! RESYNCDIS - Resynchronization Disable
 *  0b0..Resynchronization during received data word is supported.
 *  0b1..Resynchronization during received data word is disabled.
 */
#define LPUART_BAUD_RESYNCDIS(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RESYNCDIS_SHIFT)) & LPUART_BAUD_RESYNCDIS_MASK)

#define LPUART_BAUD_BOTHEDGE_MASK  (0x20000U)
#define LPUART_BAUD_BOTHEDGE_SHIFT (17U)
/*! BOTHEDGE - Both Edge Sampling
 *  0b0..Receiver samples input data using the rising edge of the baud rate clock.
 *  0b1..Receiver samples input data using the rising and falling edge of the baud rate clock.
 */
#define LPUART_BAUD_BOTHEDGE(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_BOTHEDGE_SHIFT)) & LPUART_BAUD_BOTHEDGE_MASK)

#define LPUART_BAUD_MATCFG_MASK  (0xC0000U)
#define LPUART_BAUD_MATCFG_SHIFT (18U)
/*! MATCFG - Match Configuration
 *  0b00..Address Match Wakeup
 *  0b01..Idle Match Wakeup
 *  0b10..Match On and Match Off
 *  0b11..Enables RWU on Data Match and Match On/Off for transmitter CTS input
 */
#define LPUART_BAUD_MATCFG(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MATCFG_SHIFT)) & LPUART_BAUD_MATCFG_MASK)

#define LPUART_BAUD_RIDMAE_MASK  (0x100000U)
#define LPUART_BAUD_RIDMAE_SHIFT (20U)
/*! RIDMAE - Receiver Idle DMA Enable
 *  0b0..DMA request disabled.
 *  0b1..DMA request enabled.
 */
#define LPUART_BAUD_RIDMAE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RIDMAE_SHIFT)) & LPUART_BAUD_RIDMAE_MASK)

#define LPUART_BAUD_RDMAE_MASK  (0x200000U)
#define LPUART_BAUD_RDMAE_SHIFT (21U)
/*! RDMAE - Receiver Full DMA Enable
 *  0b0..DMA request disabled.
 *  0b1..DMA request enabled.
 */
#define LPUART_BAUD_RDMAE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_RDMAE_SHIFT)) & LPUART_BAUD_RDMAE_MASK)

#define LPUART_BAUD_TDMAE_MASK  (0x800000U)
#define LPUART_BAUD_TDMAE_SHIFT (23U)
/*! TDMAE - Transmitter DMA Enable
 *  0b0..DMA request disabled.
 *  0b1..DMA request enabled.
 */
#define LPUART_BAUD_TDMAE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_TDMAE_SHIFT)) & LPUART_BAUD_TDMAE_MASK)

#define LPUART_BAUD_OSR_MASK  (0x1F000000U)
#define LPUART_BAUD_OSR_SHIFT (24U)
/*! OSR - Oversampling Ratio
 *  0b00000..Writing 0 to this field results in an oversampling ratio of 16
 *  0b00001..Reserved
 *  0b00010..Reserved
 *  0b00011..Oversampling ratio of 4, requires BOTHEDGE to be set.
 *  0b00100..Oversampling ratio of 5, requires BOTHEDGE to be set.
 *  0b00101..Oversampling ratio of 6, requires BOTHEDGE to be set.
 *  0b00110..Oversampling ratio of 7, requires BOTHEDGE to be set.
 *  0b00111..Oversampling ratio of 8.
 *  0b01000..Oversampling ratio of 9.
 *  0b01001..Oversampling ratio of 10.
 *  0b01010..Oversampling ratio of 11.
 *  0b01011..Oversampling ratio of 12.
 *  0b01100..Oversampling ratio of 13.
 *  0b01101..Oversampling ratio of 14.
 *  0b01110..Oversampling ratio of 15.
 *  0b01111..Oversampling ratio of 16.
 *  0b10000..Oversampling ratio of 17.
 *  0b10001..Oversampling ratio of 18.
 *  0b10010..Oversampling ratio of 19.
 *  0b10011..Oversampling ratio of 20.
 *  0b10100..Oversampling ratio of 21.
 *  0b10101..Oversampling ratio of 22.
 *  0b10110..Oversampling ratio of 23.
 *  0b10111..Oversampling ratio of 24.
 *  0b11000..Oversampling ratio of 25.
 *  0b11001..Oversampling ratio of 26.
 *  0b11010..Oversampling ratio of 27.
 *  0b11011..Oversampling ratio of 28.
 *  0b11100..Oversampling ratio of 29.
 *  0b11101..Oversampling ratio of 30.
 *  0b11110..Oversampling ratio of 31.
 *  0b11111..Oversampling ratio of 32.
 */
#define LPUART_BAUD_OSR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_OSR_SHIFT)) & LPUART_BAUD_OSR_MASK)

#define LPUART_BAUD_M10_MASK  (0x20000000U)
#define LPUART_BAUD_M10_SHIFT (29U)
/*! M10 - 10-bit Mode select
 *  0b0..Receiver and transmitter use 7-bit to 9-bit data characters.
 *  0b1..Receiver and transmitter use 10-bit data characters.
 */
#define LPUART_BAUD_M10(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_M10_SHIFT)) & LPUART_BAUD_M10_MASK)

#define LPUART_BAUD_MAEN2_MASK  (0x40000000U)
#define LPUART_BAUD_MAEN2_SHIFT (30U)
/*! MAEN2 - Match Address Mode Enable 2
 *  0b0..Normal operation.
 *  0b1..Enables automatic address matching or data matching mode for MATCH[MA2].
 */
#define LPUART_BAUD_MAEN2(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN2_SHIFT)) & LPUART_BAUD_MAEN2_MASK)

#define LPUART_BAUD_MAEN1_MASK  (0x80000000U)
#define LPUART_BAUD_MAEN1_SHIFT (31U)
/*! MAEN1 - Match Address Mode Enable 1
 *  0b0..Normal operation.
 *  0b1..Enables automatic address matching or data matching mode for MATCH[MA1].
 */
#define LPUART_BAUD_MAEN1(x) (((uint32_t)(((uint32_t)(x)) << LPUART_BAUD_MAEN1_SHIFT)) & LPUART_BAUD_MAEN1_MASK)
/*! @} */

/*! @name STAT - LPUART Status Register */
/*! @{ */

#define LPUART_STAT_LBKFE_MASK  (0x1U)
#define LPUART_STAT_LBKFE_SHIFT (0U)
/*! LBKFE - LIN Break Flag Enable
 *  0b0..LIN break detect is disabled.
 *  0b1..LIN break detect is enabled. LIN break character is detected at length of 11 bit times (if M = 0) or 12 (if M =
 * 1) or 13 (M10 = 1).
 */
#define LPUART_STAT_LBKFE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKFE_SHIFT)) & LPUART_STAT_LBKFE_MASK)

#define LPUART_STAT_AME_MASK  (0x2U)
#define LPUART_STAT_AME_SHIFT (1U)
/*! AME - Address Mark Enable
 *  0b0..Address mark in character is MSB.
 *  0b1..Address mark in character is last bit before stop bit (or parity bit when enabled) and stored in DATA
 *       register at MSB (or MSB-1 when parity bit enabled).
 */
#define LPUART_STAT_AME(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_AME_SHIFT)) & LPUART_STAT_AME_MASK)

#define LPUART_STAT_MSF_MASK  (0x100U)
#define LPUART_STAT_MSF_SHIFT (8U)
/*! MSF - MODEM Status Flag
 *  0b0..Flag is clear
 *  0b1..Flag is set
 */
#define LPUART_STAT_MSF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MSF_SHIFT)) & LPUART_STAT_MSF_MASK)

#define LPUART_STAT_TSF_MASK  (0x200U)
#define LPUART_STAT_TSF_SHIFT (9U)
/*! TSF - Timeout Status Flag
 *  0b0..Flag is clear
 *  0b1..Flag is set
 */
#define LPUART_STAT_TSF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TSF_SHIFT)) & LPUART_STAT_TSF_MASK)

#define LPUART_STAT_MA2F_MASK  (0x4000U)
#define LPUART_STAT_MA2F_SHIFT (14U)
/*! MA2F - Match 2 Flag
 *  0b0..Received data is not equal to MA2
 *  0b1..Received data is equal to MA2
 */
#define LPUART_STAT_MA2F(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA2F_SHIFT)) & LPUART_STAT_MA2F_MASK)

#define LPUART_STAT_MA1F_MASK  (0x8000U)
#define LPUART_STAT_MA1F_SHIFT (15U)
/*! MA1F - Match 1 Flag
 *  0b0..Received data is not equal to MA1
 *  0b1..Received data is equal to MA1
 */
#define LPUART_STAT_MA1F(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MA1F_SHIFT)) & LPUART_STAT_MA1F_MASK)

#define LPUART_STAT_PF_MASK  (0x10000U)
#define LPUART_STAT_PF_SHIFT (16U)
/*! PF - Parity Error Flag
 *  0b0..No parity error.
 *  0b1..Parity error.
 */
#define LPUART_STAT_PF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_PF_SHIFT)) & LPUART_STAT_PF_MASK)

#define LPUART_STAT_FE_MASK  (0x20000U)
#define LPUART_STAT_FE_SHIFT (17U)
/*! FE - Framing Error Flag
 *  0b0..No framing error detected. This does not guarantee the framing is correct.
 *  0b1..Framing error.
 */
#define LPUART_STAT_FE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_FE_SHIFT)) & LPUART_STAT_FE_MASK)

#define LPUART_STAT_NF_MASK  (0x40000U)
#define LPUART_STAT_NF_SHIFT (18U)
/*! NF - Noise Flag
 *  0b0..No noise detected.
 *  0b1..Noise detected in the received character in the DATA register.
 */
#define LPUART_STAT_NF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_NF_SHIFT)) & LPUART_STAT_NF_MASK)

#define LPUART_STAT_OR_MASK  (0x80000U)
#define LPUART_STAT_OR_SHIFT (19U)
/*! OR - Receiver Overrun Flag
 *  0b0..No overrun.
 *  0b1..Receive overrun (new LPUART data lost).
 */
#define LPUART_STAT_OR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_OR_SHIFT)) & LPUART_STAT_OR_MASK)

#define LPUART_STAT_IDLE_MASK  (0x100000U)
#define LPUART_STAT_IDLE_SHIFT (20U)
/*! IDLE - Idle Line Flag
 *  0b0..No idle line detected.
 *  0b1..Idle line is detected.
 */
#define LPUART_STAT_IDLE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_IDLE_SHIFT)) & LPUART_STAT_IDLE_MASK)

#define LPUART_STAT_RDRF_MASK  (0x200000U)
#define LPUART_STAT_RDRF_SHIFT (21U)
/*! RDRF - Receive Data Register Full Flag
 *  0b0..Receive FIFO level is less than watermark.
 *  0b1..Receive FIFO level is equal or greater than watermark.
 */
#define LPUART_STAT_RDRF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RDRF_SHIFT)) & LPUART_STAT_RDRF_MASK)

#define LPUART_STAT_TC_MASK  (0x400000U)
#define LPUART_STAT_TC_SHIFT (22U)
/*! TC - Transmission Complete Flag
 *  0b0..Transmitter active (sending data, a preamble, or a break).
 *  0b1..Transmitter idle (transmission activity complete).
 */
#define LPUART_STAT_TC(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TC_SHIFT)) & LPUART_STAT_TC_MASK)

#define LPUART_STAT_TDRE_MASK  (0x800000U)
#define LPUART_STAT_TDRE_SHIFT (23U)
/*! TDRE - Transmit Data Register Empty Flag
 *  0b0..Transmit FIFO level is greater than watermark.
 *  0b1..Transmit FIFO level is equal or less than watermark.
 */
#define LPUART_STAT_TDRE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_TDRE_SHIFT)) & LPUART_STAT_TDRE_MASK)

#define LPUART_STAT_RAF_MASK  (0x1000000U)
#define LPUART_STAT_RAF_SHIFT (24U)
/*! RAF - Receiver Active Flag
 *  0b0..LPUART receiver idle waiting for a start bit.
 *  0b1..LPUART receiver active (RXD input not idle).
 */
#define LPUART_STAT_RAF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RAF_SHIFT)) & LPUART_STAT_RAF_MASK)

#define LPUART_STAT_LBKDE_MASK  (0x2000000U)
#define LPUART_STAT_LBKDE_SHIFT (25U)
/*! LBKDE - LIN Break Detection Enable
 *  0b0..LIN break detect is disabled, normal break character can be detected.
 *  0b1..LIN break detect is enabled. LIN break character is detected at length of 11 bit times (if M = 0) or 12 (if M =
 * 1) or 13 (M10 = 1).
 */
#define LPUART_STAT_LBKDE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDE_SHIFT)) & LPUART_STAT_LBKDE_MASK)

#define LPUART_STAT_BRK13_MASK  (0x4000000U)
#define LPUART_STAT_BRK13_SHIFT (26U)
/*! BRK13 - Break Character Generation Length
 *  0b0..Break character is transmitted with length of 9 to 13 bit times.
 *  0b1..Break character is transmitted with length of 12 to 15 bit times.
 */
#define LPUART_STAT_BRK13(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_BRK13_SHIFT)) & LPUART_STAT_BRK13_MASK)

#define LPUART_STAT_RWUID_MASK  (0x8000000U)
#define LPUART_STAT_RWUID_SHIFT (27U)
/*! RWUID - Receive Wake Up Idle Detect
 *  0b0..During receive standby state (RWU = 1), the IDLE bit does not get set upon detection of an idle
 *       character. During address match wakeup, the IDLE bit does not set when an address does not match.
 *  0b1..During receive standby state (RWU = 1), the IDLE bit gets set upon detection of an idle character. During
 *       address match wakeup, the IDLE bit does set when an address does not match.
 */
#define LPUART_STAT_RWUID(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RWUID_SHIFT)) & LPUART_STAT_RWUID_MASK)

#define LPUART_STAT_RXINV_MASK  (0x10000000U)
#define LPUART_STAT_RXINV_SHIFT (28U)
/*! RXINV - Receive Data Inversion
 *  0b0..Receive data not inverted.
 *  0b1..Receive data inverted.
 */
#define LPUART_STAT_RXINV(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXINV_SHIFT)) & LPUART_STAT_RXINV_MASK)

#define LPUART_STAT_MSBF_MASK  (0x20000000U)
#define LPUART_STAT_MSBF_SHIFT (29U)
/*! MSBF - MSB First
 *  0b0..LSB (bit0) is the first bit that is transmitted following the start bit. Further, the first bit received
 *       after the start bit is identified as bit0.
 *  0b1..MSB (identified as bit9, bit8, bit7 or bit6) is the first bit that is transmitted following the start bit
 *       depending on the setting of CTRL[M], CTRL[PE] and BAUD[M10]. .
 */
#define LPUART_STAT_MSBF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_MSBF_SHIFT)) & LPUART_STAT_MSBF_MASK)

#define LPUART_STAT_RXEDGIF_MASK  (0x40000000U)
#define LPUART_STAT_RXEDGIF_SHIFT (30U)
/*! RXEDGIF - RXD Pin Active Edge Interrupt Flag
 *  0b0..No active edge on the receive pin has occurred.
 *  0b1..An active edge on the receive pin has occurred.
 */
#define LPUART_STAT_RXEDGIF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_RXEDGIF_SHIFT)) & LPUART_STAT_RXEDGIF_MASK)

#define LPUART_STAT_LBKDIF_MASK  (0x80000000U)
#define LPUART_STAT_LBKDIF_SHIFT (31U)
/*! LBKDIF - LIN Break Detect Interrupt Flag
 *  0b0..No LIN break character has been detected.
 *  0b1..LIN break character has been detected.
 */
#define LPUART_STAT_LBKDIF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_STAT_LBKDIF_SHIFT)) & LPUART_STAT_LBKDIF_MASK)
/*! @} */

/*! @name CTRL - LPUART Control Register */
/*! @{ */

#define LPUART_CTRL_PT_MASK  (0x1U)
#define LPUART_CTRL_PT_SHIFT (0U)
/*! PT - Parity Type
 *  0b0..Even parity.
 *  0b1..Odd parity.
 */
#define LPUART_CTRL_PT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PT_SHIFT)) & LPUART_CTRL_PT_MASK)

#define LPUART_CTRL_PE_MASK  (0x2U)
#define LPUART_CTRL_PE_SHIFT (1U)
/*! PE - Parity Enable
 *  0b0..No hardware parity generation or checking.
 *  0b1..Parity enabled.
 */
#define LPUART_CTRL_PE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PE_SHIFT)) & LPUART_CTRL_PE_MASK)

#define LPUART_CTRL_ILT_MASK  (0x4U)
#define LPUART_CTRL_ILT_SHIFT (2U)
/*! ILT - Idle Line Type Select
 *  0b0..Idle character bit count starts after start bit.
 *  0b1..Idle character bit count starts after stop bit.
 */
#define LPUART_CTRL_ILT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILT_SHIFT)) & LPUART_CTRL_ILT_MASK)

#define LPUART_CTRL_WAKE_MASK  (0x8U)
#define LPUART_CTRL_WAKE_SHIFT (3U)
/*! WAKE - Receiver Wakeup Method Select
 *  0b0..Configures RWU for idle-line wakeup.
 *  0b1..Configures RWU with address-mark wakeup.
 */
#define LPUART_CTRL_WAKE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_WAKE_SHIFT)) & LPUART_CTRL_WAKE_MASK)

#define LPUART_CTRL_M_MASK  (0x10U)
#define LPUART_CTRL_M_SHIFT (4U)
/*! M - 9-Bit or 8-Bit Mode Select
 *  0b0..Receiver and transmitter use 8-bit data characters.
 *  0b1..Receiver and transmitter use 9-bit data characters.
 */
#define LPUART_CTRL_M(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M_SHIFT)) & LPUART_CTRL_M_MASK)

#define LPUART_CTRL_RSRC_MASK  (0x20U)
#define LPUART_CTRL_RSRC_SHIFT (5U)
/*! RSRC - Receiver Source Select
 *  0b0..Provided LOOPS is set, RSRC is cleared, selects internal loop back mode and the LPUART does not use the
 *       RXD pin. The CTS_B pin is not used and internally driven by the RTS_B output. The DSR_B pin is not used and
 *       internally driven by the DTR_B output.
 *  0b1..Single-wire LPUART mode where the TXD pin is connected to the transmitter output and receiver input.
 */
#define LPUART_CTRL_RSRC(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RSRC_SHIFT)) & LPUART_CTRL_RSRC_MASK)

#define LPUART_CTRL_DOZEEN_MASK  (0x40U)
#define LPUART_CTRL_DOZEEN_SHIFT (6U)
/*! DOZEEN - Doze Enable
 *  0b0..LPUART is enabled in Doze mode.
 *  0b1..LPUART is disabled in Doze mode , but remains active when not in Doze mode .
 */
#define LPUART_CTRL_DOZEEN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_DOZEEN_SHIFT)) & LPUART_CTRL_DOZEEN_MASK)

#define LPUART_CTRL_LOOPS_MASK  (0x80U)
#define LPUART_CTRL_LOOPS_SHIFT (7U)
/*! LOOPS - Loop Mode Select
 *  0b0..Normal operation - RXD and TXD use separate pins.
 *  0b1..Loop mode or single-wire mode where transmitter outputs are internally connected to receiver input (see RSRC
 * bit).
 */
#define LPUART_CTRL_LOOPS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_LOOPS_SHIFT)) & LPUART_CTRL_LOOPS_MASK)

#define LPUART_CTRL_IDLECFG_MASK  (0x700U)
#define LPUART_CTRL_IDLECFG_SHIFT (8U)
/*! IDLECFG - Idle Configuration
 *  0b000..1 idle character
 *  0b001..2 idle characters
 *  0b010..4 idle characters
 *  0b011..8 idle characters
 *  0b100..16 idle characters
 *  0b101..32 idle characters
 *  0b110..64 idle characters
 *  0b111..128 idle characters
 */
#define LPUART_CTRL_IDLECFG(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_IDLECFG_SHIFT)) & LPUART_CTRL_IDLECFG_MASK)

#define LPUART_CTRL_M7_MASK  (0x800U)
#define LPUART_CTRL_M7_SHIFT (11U)
/*! M7 - 7-Bit Mode Select
 *  0b0..Receiver and transmitter use 8-bit to 10-bit data characters.
 *  0b1..Receiver and transmitter use 7-bit data characters.
 */
#define LPUART_CTRL_M7(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_M7_SHIFT)) & LPUART_CTRL_M7_MASK)

#define LPUART_CTRL_MA2IE_MASK  (0x4000U)
#define LPUART_CTRL_MA2IE_SHIFT (14U)
/*! MA2IE - Match 2 Interrupt Enable
 *  0b0..MA2F interrupt disabled
 *  0b1..MA2F interrupt enabled
 */
#define LPUART_CTRL_MA2IE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA2IE_SHIFT)) & LPUART_CTRL_MA2IE_MASK)

#define LPUART_CTRL_MA1IE_MASK  (0x8000U)
#define LPUART_CTRL_MA1IE_SHIFT (15U)
/*! MA1IE - Match 1 Interrupt Enable
 *  0b0..MA1F interrupt disabled
 *  0b1..MA1F interrupt enabled
 */
#define LPUART_CTRL_MA1IE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_MA1IE_SHIFT)) & LPUART_CTRL_MA1IE_MASK)

#define LPUART_CTRL_SBK_MASK  (0x10000U)
#define LPUART_CTRL_SBK_SHIFT (16U)
/*! SBK - Send Break
 *  0b0..Normal transmitter operation.
 *  0b1..Queue break character(s) to be sent.
 */
#define LPUART_CTRL_SBK(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_SBK_SHIFT)) & LPUART_CTRL_SBK_MASK)

#define LPUART_CTRL_RWU_MASK  (0x20000U)
#define LPUART_CTRL_RWU_SHIFT (17U)
/*! RWU - Receiver Wakeup Control
 *  0b0..Normal receiver operation.
 *  0b1..LPUART receiver in standby waiting for wakeup condition.
 */
#define LPUART_CTRL_RWU(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RWU_SHIFT)) & LPUART_CTRL_RWU_MASK)

#define LPUART_CTRL_RE_MASK  (0x40000U)
#define LPUART_CTRL_RE_SHIFT (18U)
/*! RE - Receiver Enable
 *  0b0..Receiver disabled.
 *  0b1..Receiver enabled.
 */
#define LPUART_CTRL_RE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RE_SHIFT)) & LPUART_CTRL_RE_MASK)

#define LPUART_CTRL_TE_MASK  (0x80000U)
#define LPUART_CTRL_TE_SHIFT (19U)
/*! TE - Transmitter Enable
 *  0b0..Transmitter disabled.
 *  0b1..Transmitter enabled.
 */
#define LPUART_CTRL_TE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TE_SHIFT)) & LPUART_CTRL_TE_MASK)

#define LPUART_CTRL_ILIE_MASK  (0x100000U)
#define LPUART_CTRL_ILIE_SHIFT (20U)
/*! ILIE - Idle Line Interrupt Enable
 *  0b0..Hardware interrupts from IDLE disabled; use polling.
 *  0b1..Hardware interrupt is requested when IDLE flag is 1.
 */
#define LPUART_CTRL_ILIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ILIE_SHIFT)) & LPUART_CTRL_ILIE_MASK)

#define LPUART_CTRL_RIE_MASK  (0x200000U)
#define LPUART_CTRL_RIE_SHIFT (21U)
/*! RIE - Receiver Interrupt Enable
 *  0b0..Hardware interrupts from RDRF disabled.
 *  0b1..Hardware interrupt is requested when RDRF flag is 1.
 */
#define LPUART_CTRL_RIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_RIE_SHIFT)) & LPUART_CTRL_RIE_MASK)

#define LPUART_CTRL_TCIE_MASK  (0x400000U)
#define LPUART_CTRL_TCIE_SHIFT (22U)
/*! TCIE - Transmission Complete Interrupt Enable for
 *  0b0..Hardware interrupts from TC disabled.
 *  0b1..Hardware interrupt is requested when TC flag is 1.
 */
#define LPUART_CTRL_TCIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TCIE_SHIFT)) & LPUART_CTRL_TCIE_MASK)

#define LPUART_CTRL_TIE_MASK  (0x800000U)
#define LPUART_CTRL_TIE_SHIFT (23U)
/*! TIE - Transmit Interrupt Enable
 *  0b0..Hardware interrupts from TDRE disabled.
 *  0b1..Hardware interrupt is requested when TDRE flag is 1.
 */
#define LPUART_CTRL_TIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TIE_SHIFT)) & LPUART_CTRL_TIE_MASK)

#define LPUART_CTRL_PEIE_MASK  (0x1000000U)
#define LPUART_CTRL_PEIE_SHIFT (24U)
/*! PEIE - Parity Error Interrupt Enable
 *  0b0..PF interrupts disabled; use polling).
 *  0b1..Hardware interrupt is requested when PF is set.
 */
#define LPUART_CTRL_PEIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_PEIE_SHIFT)) & LPUART_CTRL_PEIE_MASK)

#define LPUART_CTRL_FEIE_MASK  (0x2000000U)
#define LPUART_CTRL_FEIE_SHIFT (25U)
/*! FEIE - Framing Error Interrupt Enable
 *  0b0..FE interrupts disabled; use polling.
 *  0b1..Hardware interrupt is requested when FE is set.
 */
#define LPUART_CTRL_FEIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_FEIE_SHIFT)) & LPUART_CTRL_FEIE_MASK)

#define LPUART_CTRL_NEIE_MASK  (0x4000000U)
#define LPUART_CTRL_NEIE_SHIFT (26U)
/*! NEIE - Noise Error Interrupt Enable
 *  0b0..NF interrupts disabled; use polling.
 *  0b1..Hardware interrupt is requested when NF is set.
 */
#define LPUART_CTRL_NEIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_NEIE_SHIFT)) & LPUART_CTRL_NEIE_MASK)

#define LPUART_CTRL_ORIE_MASK  (0x8000000U)
#define LPUART_CTRL_ORIE_SHIFT (27U)
/*! ORIE - Overrun Interrupt Enable
 *  0b0..OR interrupts disabled; use polling.
 *  0b1..Hardware interrupt is requested when OR is set.
 */
#define LPUART_CTRL_ORIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_ORIE_SHIFT)) & LPUART_CTRL_ORIE_MASK)

#define LPUART_CTRL_TXINV_MASK  (0x10000000U)
#define LPUART_CTRL_TXINV_SHIFT (28U)
/*! TXINV - Transmit Data Inversion
 *  0b0..Transmit data not inverted.
 *  0b1..Transmit data inverted.
 */
#define LPUART_CTRL_TXINV(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXINV_SHIFT)) & LPUART_CTRL_TXINV_MASK)

#define LPUART_CTRL_TXDIR_MASK  (0x20000000U)
#define LPUART_CTRL_TXDIR_SHIFT (29U)
/*! TXDIR - TXD Pin Direction in Single-Wire Mode
 *  0b0..TXD pin is an input in single-wire mode.
 *  0b1..TXD pin is an output in single-wire mode.
 */
#define LPUART_CTRL_TXDIR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_TXDIR_SHIFT)) & LPUART_CTRL_TXDIR_MASK)

#define LPUART_CTRL_R9T8_MASK  (0x40000000U)
#define LPUART_CTRL_R9T8_SHIFT (30U)
/*! R9T8 - Receive Bit 9 / Transmit Bit 8
 */
#define LPUART_CTRL_R9T8(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R9T8_SHIFT)) & LPUART_CTRL_R9T8_MASK)

#define LPUART_CTRL_R8T9_MASK  (0x80000000U)
#define LPUART_CTRL_R8T9_SHIFT (31U)
/*! R8T9 - Receive Bit 8 / Transmit Bit 9
 */
#define LPUART_CTRL_R8T9(x) (((uint32_t)(((uint32_t)(x)) << LPUART_CTRL_R8T9_SHIFT)) & LPUART_CTRL_R8T9_MASK)
/*! @} */

/*! @name DATA - LPUART Data Register */
/*! @{ */

#define LPUART_DATA_R0T0_MASK  (0x1U)
#define LPUART_DATA_R0T0_SHIFT (0U)
/*! R0T0 - R0T0
 */
#define LPUART_DATA_R0T0(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R0T0_SHIFT)) & LPUART_DATA_R0T0_MASK)

#define LPUART_DATA_R1T1_MASK  (0x2U)
#define LPUART_DATA_R1T1_SHIFT (1U)
/*! R1T1 - R1T1
 */
#define LPUART_DATA_R1T1(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R1T1_SHIFT)) & LPUART_DATA_R1T1_MASK)

#define LPUART_DATA_R2T2_MASK  (0x4U)
#define LPUART_DATA_R2T2_SHIFT (2U)
/*! R2T2 - R2T2
 */
#define LPUART_DATA_R2T2(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R2T2_SHIFT)) & LPUART_DATA_R2T2_MASK)

#define LPUART_DATA_R3T3_MASK  (0x8U)
#define LPUART_DATA_R3T3_SHIFT (3U)
/*! R3T3 - R3T3
 */
#define LPUART_DATA_R3T3(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R3T3_SHIFT)) & LPUART_DATA_R3T3_MASK)

#define LPUART_DATA_R4T4_MASK  (0x10U)
#define LPUART_DATA_R4T4_SHIFT (4U)
/*! R4T4 - R4T4
 */
#define LPUART_DATA_R4T4(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R4T4_SHIFT)) & LPUART_DATA_R4T4_MASK)

#define LPUART_DATA_R5T5_MASK  (0x20U)
#define LPUART_DATA_R5T5_SHIFT (5U)
/*! R5T5 - R5T5
 */
#define LPUART_DATA_R5T5(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R5T5_SHIFT)) & LPUART_DATA_R5T5_MASK)

#define LPUART_DATA_R6T6_MASK  (0x40U)
#define LPUART_DATA_R6T6_SHIFT (6U)
/*! R6T6 - R6T6
 */
#define LPUART_DATA_R6T6(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R6T6_SHIFT)) & LPUART_DATA_R6T6_MASK)

#define LPUART_DATA_R7T7_MASK  (0x80U)
#define LPUART_DATA_R7T7_SHIFT (7U)
/*! R7T7 - R7T7
 */
#define LPUART_DATA_R7T7(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R7T7_SHIFT)) & LPUART_DATA_R7T7_MASK)

#define LPUART_DATA_R8T8_MASK  (0x100U)
#define LPUART_DATA_R8T8_SHIFT (8U)
/*! R8T8 - R8T8
 */
#define LPUART_DATA_R8T8(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R8T8_SHIFT)) & LPUART_DATA_R8T8_MASK)

#define LPUART_DATA_R9T9_MASK  (0x200U)
#define LPUART_DATA_R9T9_SHIFT (9U)
/*! R9T9 - R9T9
 */
#define LPUART_DATA_R9T9(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_R9T9_SHIFT)) & LPUART_DATA_R9T9_MASK)

#define LPUART_DATA_LINBRK_MASK  (0x400U)
#define LPUART_DATA_LINBRK_SHIFT (10U)
/*! LINBRK - LIN Break
 *  0b0..Receiver did not detect LIN break before this character, or LIN break detect circuitry disabled.
 *  0b1..Receiver detected a LIN break before receiving this character.
 */
#define LPUART_DATA_LINBRK(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_LINBRK_SHIFT)) & LPUART_DATA_LINBRK_MASK)

#define LPUART_DATA_IDLINE_MASK  (0x800U)
#define LPUART_DATA_IDLINE_SHIFT (11U)
/*! IDLINE - Idle Line
 *  0b0..Receiver was not idle before receiving this character.
 *  0b1..Receiver was idle before receiving this character.
 */
#define LPUART_DATA_IDLINE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_IDLINE_SHIFT)) & LPUART_DATA_IDLINE_MASK)

#define LPUART_DATA_RXEMPT_MASK  (0x1000U)
#define LPUART_DATA_RXEMPT_SHIFT (12U)
/*! RXEMPT - Receive Buffer Empty
 *  0b0..Receive buffer contains valid data.
 *  0b1..Receive buffer is empty, data returned on read is not valid.
 */
#define LPUART_DATA_RXEMPT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_RXEMPT_SHIFT)) & LPUART_DATA_RXEMPT_MASK)

#define LPUART_DATA_FRETSC_MASK  (0x2000U)
#define LPUART_DATA_FRETSC_SHIFT (13U)
/*! FRETSC - Frame Error / Transmit Special Character
 *  0b0..The dataword is received without a frame error on read, or transmit a normal character on write.
 *  0b1..The dataword is received with a frame error, or transmit an idle or break character on transmit.
 */
#define LPUART_DATA_FRETSC(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_FRETSC_SHIFT)) & LPUART_DATA_FRETSC_MASK)

#define LPUART_DATA_PARITYE_MASK  (0x4000U)
#define LPUART_DATA_PARITYE_SHIFT (14U)
/*! PARITYE - Parity Error
 *  0b0..The dataword is received without a parity error.
 *  0b1..The dataword is received with a parity error.
 */
#define LPUART_DATA_PARITYE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_PARITYE_SHIFT)) & LPUART_DATA_PARITYE_MASK)

#define LPUART_DATA_NOISY_MASK  (0x8000U)
#define LPUART_DATA_NOISY_SHIFT (15U)
/*! NOISY - Noisy Data Received
 *  0b0..The dataword is received without noise.
 *  0b1..The data is received with noise.
 */
#define LPUART_DATA_NOISY(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATA_NOISY_SHIFT)) & LPUART_DATA_NOISY_MASK)
/*! @} */

/*! @name MATCH - LPUART Match Address Register */
/*! @{ */

#define LPUART_MATCH_MA1_MASK  (0x3FFU)
#define LPUART_MATCH_MA1_SHIFT (0U)
/*! MA1 - Match Address 1
 */
#define LPUART_MATCH_MA1(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA1_SHIFT)) & LPUART_MATCH_MA1_MASK)

#define LPUART_MATCH_MA2_MASK  (0x3FF0000U)
#define LPUART_MATCH_MA2_SHIFT (16U)
/*! MA2 - Match Address 2
 */
#define LPUART_MATCH_MA2(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MATCH_MA2_SHIFT)) & LPUART_MATCH_MA2_MASK)
/*! @} */

/*! @name MODIR - LPUART Modem IrDA Register */
/*! @{ */

#define LPUART_MODIR_TXCTSE_MASK  (0x1U)
#define LPUART_MODIR_TXCTSE_SHIFT (0U)
/*! TXCTSE - Transmitter clear-to-send enable
 *  0b0..CTS has no effect on the transmitter.
 *  0b1..Enables clear-to-send operation. The transmitter checks the state of CTS each time it is ready to send a
 *       character. If CTS is asserted, the character is sent. If CTS is deasserted, the signal TXD remains in the
 *       mark state and transmission is delayed until CTS is asserted. Changes in CTS as a character is being sent
 *       do not affect its transmission.
 */
#define LPUART_MODIR_TXCTSE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSE_SHIFT)) & LPUART_MODIR_TXCTSE_MASK)

#define LPUART_MODIR_TXRTSE_MASK  (0x2U)
#define LPUART_MODIR_TXRTSE_SHIFT (1U)
/*! TXRTSE - Transmitter request-to-send enable
 *  0b0..The transmitter has no effect on RTS.
 *  0b1..When a character is placed into an empty transmit shift register, RTS asserts one bit time before the
 *       start bit is transmitted. RTS deasserts one bit time after all characters in the transmitter FIFO and shift
 *       register are completely sent, including the last stop bit.
 */
#define LPUART_MODIR_TXRTSE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSE_SHIFT)) & LPUART_MODIR_TXRTSE_MASK)

#define LPUART_MODIR_TXRTSPOL_MASK  (0x4U)
#define LPUART_MODIR_TXRTSPOL_SHIFT (2U)
/*! TXRTSPOL - Transmitter request-to-send polarity
 *  0b0..Transmitter RTS is active low.
 *  0b1..Transmitter RTS is active high.
 */
#define LPUART_MODIR_TXRTSPOL(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXRTSPOL_SHIFT)) & LPUART_MODIR_TXRTSPOL_MASK)

#define LPUART_MODIR_RXRTSE_MASK  (0x8U)
#define LPUART_MODIR_RXRTSE_SHIFT (3U)
/*! RXRTSE - Receiver request-to-send enable
 *  0b0..The receiver has no effect on RTS.
 *  0b1..RTS is deasserted if the receiver data register is full or a start bit has been detected that would cause
 *       the receiver data register to become full. RTS is asserted if the receiver data register is not full and
 *       has not detected a start bit that would cause the receiver data register to become full.
 */
#define LPUART_MODIR_RXRTSE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RXRTSE_SHIFT)) & LPUART_MODIR_RXRTSE_MASK)

#define LPUART_MODIR_TXCTSC_MASK  (0x10U)
#define LPUART_MODIR_TXCTSC_SHIFT (4U)
/*! TXCTSC - Transmit CTS Configuration
 *  0b0..CTS input is sampled at the start of each character.
 *  0b1..CTS input is sampled when the transmitter is idle.
 */
#define LPUART_MODIR_TXCTSC(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSC_SHIFT)) & LPUART_MODIR_TXCTSC_MASK)

#define LPUART_MODIR_TXCTSSRC_MASK  (0x20U)
#define LPUART_MODIR_TXCTSSRC_SHIFT (5U)
/*! TXCTSSRC - Transmit CTS Source
 *  0b0..CTS input is the CTS_B pin.
 *  0b1..CTS input is an internal connection to the receiver address match result.
 */
#define LPUART_MODIR_TXCTSSRC(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TXCTSSRC_SHIFT)) & LPUART_MODIR_TXCTSSRC_MASK)

#define LPUART_MODIR_RTSWATER_MASK  (0xF00U)
#define LPUART_MODIR_RTSWATER_SHIFT (8U)
/*! RTSWATER - Receive RTS Configuration
 */
#define LPUART_MODIR_RTSWATER(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_RTSWATER_SHIFT)) & LPUART_MODIR_RTSWATER_MASK)

#define LPUART_MODIR_TNP_MASK  (0x30000U)
#define LPUART_MODIR_TNP_SHIFT (16U)
/*! TNP - Transmitter narrow pulse
 *  0b00..1/OSR.
 *  0b01..2/OSR.
 *  0b10..3/OSR.
 *  0b11..4/OSR.
 */
#define LPUART_MODIR_TNP(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_TNP_SHIFT)) & LPUART_MODIR_TNP_MASK)

#define LPUART_MODIR_IREN_MASK  (0x40000U)
#define LPUART_MODIR_IREN_SHIFT (18U)
/*! IREN - Infrared enable
 *  0b0..IR disabled.
 *  0b1..IR enabled.
 */
#define LPUART_MODIR_IREN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MODIR_IREN_SHIFT)) & LPUART_MODIR_IREN_MASK)
/*! @} */

/*! @name FIFO - LPUART FIFO Register */
/*! @{ */

#define LPUART_FIFO_RXFIFOSIZE_MASK  (0x7U)
#define LPUART_FIFO_RXFIFOSIZE_SHIFT (0U)
/*! RXFIFOSIZE - Receive FIFO Buffer Depth
 *  0b000..Receive FIFO/Buffer depth = 1 dataword.
 *  0b001..Receive FIFO/Buffer depth = 4 datawords.
 *  0b010..Receive FIFO/Buffer depth = 8 datawords.
 *  0b011..Receive FIFO/Buffer depth = 16 datawords.
 *  0b100..Receive FIFO/Buffer depth = 32 datawords.
 *  0b101..Receive FIFO/Buffer depth = 64 datawords.
 *  0b110..Receive FIFO/Buffer depth = 128 datawords.
 *  0b111..Receive FIFO/Buffer depth = 256 datawords.
 */
#define LPUART_FIFO_RXFIFOSIZE(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFIFOSIZE_SHIFT)) & LPUART_FIFO_RXFIFOSIZE_MASK)

#define LPUART_FIFO_RXFE_MASK  (0x8U)
#define LPUART_FIFO_RXFE_SHIFT (3U)
/*! RXFE - Receive FIFO Enable
 *  0b0..Receive FIFO is not enabled. Buffer depth is 1.
 *  0b1..Receive FIFO is enabled. Buffer depth is indicted by RXFIFOSIZE.
 */
#define LPUART_FIFO_RXFE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFE_SHIFT)) & LPUART_FIFO_RXFE_MASK)

#define LPUART_FIFO_TXFIFOSIZE_MASK  (0x70U)
#define LPUART_FIFO_TXFIFOSIZE_SHIFT (4U)
/*! TXFIFOSIZE - Transmit FIFO Buffer Depth
 *  0b000..Transmit FIFO/Buffer depth = 1 dataword.
 *  0b001..Transmit FIFO/Buffer depth = 4 datawords.
 *  0b010..Transmit FIFO/Buffer depth = 8 datawords.
 *  0b011..Transmit FIFO/Buffer depth = 16 datawords.
 *  0b100..Transmit FIFO/Buffer depth = 32 datawords.
 *  0b101..Transmit FIFO/Buffer depth = 64 datawords.
 *  0b110..Transmit FIFO/Buffer depth = 128 datawords.
 *  0b111..Transmit FIFO/Buffer depth = 256 datawords
 */
#define LPUART_FIFO_TXFIFOSIZE(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFIFOSIZE_SHIFT)) & LPUART_FIFO_TXFIFOSIZE_MASK)

#define LPUART_FIFO_TXFE_MASK  (0x80U)
#define LPUART_FIFO_TXFE_SHIFT (7U)
/*! TXFE - Transmit FIFO Enable
 *  0b0..Transmit FIFO is not enabled. Buffer depth is 1.
 *  0b1..Transmit FIFO is enabled. Buffer depth is indicated by TXFIFOSIZE.
 */
#define LPUART_FIFO_TXFE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFE_SHIFT)) & LPUART_FIFO_TXFE_MASK)

#define LPUART_FIFO_RXUFE_MASK  (0x100U)
#define LPUART_FIFO_RXUFE_SHIFT (8U)
/*! RXUFE - Receive FIFO Underflow Interrupt Enable
 *  0b0..RXUF flag does not generate an interrupt to the host.
 *  0b1..RXUF flag generates an interrupt to the host.
 */
#define LPUART_FIFO_RXUFE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUFE_SHIFT)) & LPUART_FIFO_RXUFE_MASK)

#define LPUART_FIFO_TXOFE_MASK  (0x200U)
#define LPUART_FIFO_TXOFE_SHIFT (9U)
/*! TXOFE - Transmit FIFO Overflow Interrupt Enable
 *  0b0..TXOF flag does not generate an interrupt to the host.
 *  0b1..TXOF flag generates an interrupt to the host.
 */
#define LPUART_FIFO_TXOFE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOFE_SHIFT)) & LPUART_FIFO_TXOFE_MASK)

#define LPUART_FIFO_RXIDEN_MASK  (0x1C00U)
#define LPUART_FIFO_RXIDEN_SHIFT (10U)
/*! RXIDEN - Receiver Idle Empty Enable
 *  0b000..Disable RDRF assertion due to partially filled FIFO when receiver is idle.
 *  0b001..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 1 character.
 *  0b010..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 2 characters.
 *  0b011..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 4 characters.
 *  0b100..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 8 characters.
 *  0b101..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 16 characters.
 *  0b110..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 32 characters.
 *  0b111..Enable RDRF assertion due to partially filled FIFO when receiver is idle for 64 characters.
 */
#define LPUART_FIFO_RXIDEN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXIDEN_SHIFT)) & LPUART_FIFO_RXIDEN_MASK)

#define LPUART_FIFO_RXFLUSH_MASK  (0x4000U)
#define LPUART_FIFO_RXFLUSH_SHIFT (14U)
/*! RXFLUSH - Receive FIFO Flush
 *  0b0..No flush operation occurs.
 *  0b1..All data in the receive FIFO/buffer is cleared out.
 */
#define LPUART_FIFO_RXFLUSH(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXFLUSH_SHIFT)) & LPUART_FIFO_RXFLUSH_MASK)

#define LPUART_FIFO_TXFLUSH_MASK  (0x8000U)
#define LPUART_FIFO_TXFLUSH_SHIFT (15U)
/*! TXFLUSH - Transmit FIFO Flush
 *  0b0..No flush operation occurs.
 *  0b1..All data in the transmit FIFO is cleared out.
 */
#define LPUART_FIFO_TXFLUSH(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXFLUSH_SHIFT)) & LPUART_FIFO_TXFLUSH_MASK)

#define LPUART_FIFO_RXUF_MASK  (0x10000U)
#define LPUART_FIFO_RXUF_SHIFT (16U)
/*! RXUF - Receiver FIFO Underflow Flag
 *  0b0..No receive FIFO underflow has occurred since the last time the flag was cleared.
 *  0b1..At least one receive FIFO underflow has occurred since the last time the flag was cleared.
 */
#define LPUART_FIFO_RXUF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXUF_SHIFT)) & LPUART_FIFO_RXUF_MASK)

#define LPUART_FIFO_TXOF_MASK  (0x20000U)
#define LPUART_FIFO_TXOF_SHIFT (17U)
/*! TXOF - Transmitter FIFO Overflow Flag
 *  0b0..No transmit FIFO overflow has occurred since the last time the flag was cleared.
 *  0b1..At least one transmit FIFO overflow has occurred since the last time the flag was cleared.
 */
#define LPUART_FIFO_TXOF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXOF_SHIFT)) & LPUART_FIFO_TXOF_MASK)

#define LPUART_FIFO_RXEMPT_MASK  (0x400000U)
#define LPUART_FIFO_RXEMPT_SHIFT (22U)
/*! RXEMPT - Receive FIFO/Buffer Empty
 *  0b0..Receive buffer is not empty.
 *  0b1..Receive buffer is empty.
 */
#define LPUART_FIFO_RXEMPT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_RXEMPT_SHIFT)) & LPUART_FIFO_RXEMPT_MASK)

#define LPUART_FIFO_TXEMPT_MASK  (0x800000U)
#define LPUART_FIFO_TXEMPT_SHIFT (23U)
/*! TXEMPT - Transmit FIFO/Buffer Empty
 *  0b0..Transmit buffer is not empty.
 *  0b1..Transmit buffer is empty.
 */
#define LPUART_FIFO_TXEMPT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_FIFO_TXEMPT_SHIFT)) & LPUART_FIFO_TXEMPT_MASK)
/*! @} */

/*! @name WATER - LPUART Watermark Register */
/*! @{ */

#define LPUART_WATER_TXWATER_MASK  (0xFU)
#define LPUART_WATER_TXWATER_SHIFT (0U)
/*! TXWATER - Transmit Watermark
 */
#define LPUART_WATER_TXWATER(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXWATER_SHIFT)) & LPUART_WATER_TXWATER_MASK)

#define LPUART_WATER_TXCOUNT_MASK  (0x1F00U)
#define LPUART_WATER_TXCOUNT_SHIFT (8U)
/*! TXCOUNT - Transmit Counter
 */
#define LPUART_WATER_TXCOUNT(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_TXCOUNT_SHIFT)) & LPUART_WATER_TXCOUNT_MASK)

#define LPUART_WATER_RXWATER_MASK  (0xF0000U)
#define LPUART_WATER_RXWATER_SHIFT (16U)
/*! RXWATER - Receive Watermark
 */
#define LPUART_WATER_RXWATER(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXWATER_SHIFT)) & LPUART_WATER_RXWATER_MASK)

#define LPUART_WATER_RXCOUNT_MASK  (0x1F000000U)
#define LPUART_WATER_RXCOUNT_SHIFT (24U)
/*! RXCOUNT - Receive Counter
 */
#define LPUART_WATER_RXCOUNT(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_WATER_RXCOUNT_SHIFT)) & LPUART_WATER_RXCOUNT_MASK)
/*! @} */

/*! @name DATARO - Data read-only Register */
/*! @{ */

#define LPUART_DATARO_DATA_MASK  (0xFFFFU)
#define LPUART_DATARO_DATA_SHIFT (0U)
/*! DATA - Receive Data
 */
#define LPUART_DATARO_DATA(x) (((uint32_t)(((uint32_t)(x)) << LPUART_DATARO_DATA_SHIFT)) & LPUART_DATARO_DATA_MASK)
/*! @} */

/*! @name MCR - MODEM Control Register */
/*! @{ */

#define LPUART_MCR_CTS_MASK  (0x1U)
#define LPUART_MCR_CTS_SHIFT (0U)
/*! CTS - Clear To Send
 *  0b0..The CTS_B interrupt disabled
 *  0b1..The CTS_B interrupt enabled
 */
#define LPUART_MCR_CTS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_CTS_SHIFT)) & LPUART_MCR_CTS_MASK)

#define LPUART_MCR_DSR_MASK  (0x2U)
#define LPUART_MCR_DSR_SHIFT (1U)
/*! DSR - Data Set Ready
 *  0b0..The DSR_B interrupt disabled
 *  0b1..The DSR_B interrupt enabled
 */
#define LPUART_MCR_DSR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_DSR_SHIFT)) & LPUART_MCR_DSR_MASK)

#define LPUART_MCR_RIN_MASK  (0x4U)
#define LPUART_MCR_RIN_SHIFT (2U)
/*! RIN - Ring Indicator
 *  0b0..The RIN_B interrupt disabled
 *  0b1..The RIN_B interrupt enabled
 */
#define LPUART_MCR_RIN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_RIN_SHIFT)) & LPUART_MCR_RIN_MASK)

#define LPUART_MCR_DCD_MASK  (0x8U)
#define LPUART_MCR_DCD_SHIFT (3U)
/*! DCD - Data Carrier Detect
 *  0b0..The DCD_B interrupt disabled
 *  0b1..The DCD_B interrupt enabled
 */
#define LPUART_MCR_DCD(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_DCD_SHIFT)) & LPUART_MCR_DCD_MASK)

#define LPUART_MCR_DTR_MASK  (0x100U)
#define LPUART_MCR_DTR_SHIFT (8U)
/*! DTR - Data Terminal Ready
 *  0b0..Default state of DTR_B is logic one
 *  0b1..Default state of DTR_B is logic zero
 */
#define LPUART_MCR_DTR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_DTR_SHIFT)) & LPUART_MCR_DTR_MASK)

#define LPUART_MCR_RTS_MASK  (0x200U)
#define LPUART_MCR_RTS_SHIFT (9U)
/*! RTS - Request To Send
 *  0b0..Default state of RTS_B is logic one
 *  0b1..Default state of RTS_B is logic zero
 */
#define LPUART_MCR_RTS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MCR_RTS_SHIFT)) & LPUART_MCR_RTS_MASK)
/*! @} */

/*! @name MSR - MODEM Status Register */
/*! @{ */

#define LPUART_MSR_DCTS_MASK  (0x1U)
#define LPUART_MSR_DCTS_SHIFT (0U)
/*! DCTS - Delta Clear To Send
 *  0b0..The CTS_B pin has not changed state
 *  0b1..The CTS_B pin has changed state
 */
#define LPUART_MSR_DCTS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DCTS_SHIFT)) & LPUART_MSR_DCTS_MASK)

#define LPUART_MSR_DDSR_MASK  (0x2U)
#define LPUART_MSR_DDSR_SHIFT (1U)
/*! DDSR - Delta Data Set Ready
 *  0b0..The DSR_B pin has not changed state
 *  0b1..The DSR_B pin has changed state
 */
#define LPUART_MSR_DDSR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DDSR_SHIFT)) & LPUART_MSR_DDSR_MASK)

#define LPUART_MSR_DRI_MASK  (0x4U)
#define LPUART_MSR_DRI_SHIFT (2U)
/*! DRI - Delta Ring Indicator
 *  0b0..The RIN_B pin has not changed state
 *  0b1..The RIN_B pin has changed state
 */
#define LPUART_MSR_DRI(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DRI_SHIFT)) & LPUART_MSR_DRI_MASK)

#define LPUART_MSR_DDCD_MASK  (0x8U)
#define LPUART_MSR_DDCD_SHIFT (3U)
/*! DDCD - Delta Data Carrier Detect
 *  0b0..The DCD_B pin has not changed state
 *  0b1..The DCD_B pin has changed state
 */
#define LPUART_MSR_DDCD(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DDCD_SHIFT)) & LPUART_MSR_DDCD_MASK)

#define LPUART_MSR_CTS_MASK  (0x10U)
#define LPUART_MSR_CTS_SHIFT (4U)
/*! CTS - Clear To Send
 *  0b0..The CTS_B pin is logic one
 *  0b1..The CTS_B pin is logic zero
 */
#define LPUART_MSR_CTS(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_CTS_SHIFT)) & LPUART_MSR_CTS_MASK)

#define LPUART_MSR_DSR_MASK  (0x20U)
#define LPUART_MSR_DSR_SHIFT (5U)
/*! DSR - Data Set Ready
 *  0b0..The DSR_B pin is logic one
 *  0b1..The DSR_B pin is logic zero
 */
#define LPUART_MSR_DSR(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DSR_SHIFT)) & LPUART_MSR_DSR_MASK)

#define LPUART_MSR_RIN_MASK  (0x40U)
#define LPUART_MSR_RIN_SHIFT (6U)
/*! RIN - Ring Indicator
 *  0b0..The RIN_B pin is logic one
 *  0b1..The RIN_B pin is logic zero
 */
#define LPUART_MSR_RIN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_RIN_SHIFT)) & LPUART_MSR_RIN_MASK)

#define LPUART_MSR_DCD_MASK  (0x80U)
#define LPUART_MSR_DCD_SHIFT (7U)
/*! DCD - Data Carrier Detect
 *  0b0..The DCD_B pin is logic one
 *  0b1..The DCD_B pin is logic zero
 */
#define LPUART_MSR_DCD(x) (((uint32_t)(((uint32_t)(x)) << LPUART_MSR_DCD_SHIFT)) & LPUART_MSR_DCD_MASK)
/*! @} */

/*! @name REIR - Receiver Extended Idle Register */
/*! @{ */

#define LPUART_REIR_IDTIME_MASK  (0x3FFFU)
#define LPUART_REIR_IDTIME_SHIFT (0U)
/*! IDTIME - Idle Time
 */
#define LPUART_REIR_IDTIME(x) (((uint32_t)(((uint32_t)(x)) << LPUART_REIR_IDTIME_SHIFT)) & LPUART_REIR_IDTIME_MASK)
/*! @} */

/*! @name TEIR - Transmitter Extended Idle Register */
/*! @{ */

#define LPUART_TEIR_IDTIME_MASK  (0x3FFFU)
#define LPUART_TEIR_IDTIME_SHIFT (0U)
/*! IDTIME - Idle Time
 */
#define LPUART_TEIR_IDTIME(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TEIR_IDTIME_SHIFT)) & LPUART_TEIR_IDTIME_MASK)
/*! @} */

/*! @name HDCR - Half Duplex Control Register */
/*! @{ */

#define LPUART_HDCR_TXSTALL_MASK  (0x1U)
#define LPUART_HDCR_TXSTALL_SHIFT (0U)
/*! TXSTALL - Transmit Stall
 *  0b0..No effect
 *  0b1..Transmitter will not become busy when receiver is active
 */
#define LPUART_HDCR_TXSTALL(x) (((uint32_t)(((uint32_t)(x)) << LPUART_HDCR_TXSTALL_SHIFT)) & LPUART_HDCR_TXSTALL_MASK)

#define LPUART_HDCR_RXSEL_MASK  (0x2U)
#define LPUART_HDCR_RXSEL_SHIFT (1U)
/*! RXSEL - Receive Select
 *  0b0..Receive data pin is RXD
 *  0b1..Receive data pin is TXD
 */
#define LPUART_HDCR_RXSEL(x) (((uint32_t)(((uint32_t)(x)) << LPUART_HDCR_RXSEL_SHIFT)) & LPUART_HDCR_RXSEL_MASK)

#define LPUART_HDCR_RXWRMSK_MASK  (0x4U)
#define LPUART_HDCR_RXWRMSK_SHIFT (2U)
/*! RXWRMSK - Receive FIFO Write Mask
 *  0b0..Receive FIFO writes are not masked
 *  0b1..Receive FIFO writes are masked by the transmitter RTS_B
 */
#define LPUART_HDCR_RXWRMSK(x) (((uint32_t)(((uint32_t)(x)) << LPUART_HDCR_RXWRMSK_SHIFT)) & LPUART_HDCR_RXWRMSK_MASK)

#define LPUART_HDCR_RXMSK_MASK  (0x8U)
#define LPUART_HDCR_RXMSK_SHIFT (3U)
/*! RXMSK - Receive Mask
 *  0b0..Receive data pin is not masked by the transmitter RTS_B
 *  0b1..Receive data pin is masked by the transmitter RTS_B
 */
#define LPUART_HDCR_RXMSK(x) (((uint32_t)(((uint32_t)(x)) << LPUART_HDCR_RXMSK_SHIFT)) & LPUART_HDCR_RXMSK_MASK)

#define LPUART_HDCR_RTSEXT_MASK  (0xFF00U)
#define LPUART_HDCR_RTSEXT_SHIFT (8U)
/*! RTSEXT - RTS Extended
 */
#define LPUART_HDCR_RTSEXT(x) (((uint32_t)(((uint32_t)(x)) << LPUART_HDCR_RTSEXT_SHIFT)) & LPUART_HDCR_RTSEXT_MASK)
/*! @} */

/*! @name TOCR - Timeout Control Register */
/*! @{ */

#define LPUART_TOCR_TOEN_MASK  (0xFU)
#define LPUART_TOCR_TOEN_SHIFT (0U)
/*! TOEN - Timeout Enable
 */
#define LPUART_TOCR_TOEN(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TOCR_TOEN_SHIFT)) & LPUART_TOCR_TOEN_MASK)

#define LPUART_TOCR_TOIE_MASK  (0xF00U)
#define LPUART_TOCR_TOIE_SHIFT (8U)
/*! TOIE - Timeout Interrupt Enable
 */
#define LPUART_TOCR_TOIE(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TOCR_TOIE_SHIFT)) & LPUART_TOCR_TOIE_MASK)
/*! @} */

/*! @name TOSR - Timeout Status Register */
/*! @{ */

#define LPUART_TOSR_TOZ_MASK  (0xFU)
#define LPUART_TOSR_TOZ_SHIFT (0U)
/*! TOZ - Timeout Zero
 */
#define LPUART_TOSR_TOZ(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TOSR_TOZ_SHIFT)) & LPUART_TOSR_TOZ_MASK)

#define LPUART_TOSR_TOF_MASK  (0xF00U)
#define LPUART_TOSR_TOF_SHIFT (8U)
/*! TOF - Timeout Flag
 */
#define LPUART_TOSR_TOF(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TOSR_TOF_SHIFT)) & LPUART_TOSR_TOF_MASK)
/*! @} */

/*! @name TIMEOUT - Timeout N Register */
/*! @{ */

#define LPUART_TIMEOUT_TIMEOUT_MASK  (0x3FFFU)
#define LPUART_TIMEOUT_TIMEOUT_SHIFT (0U)
/*! TIMEOUT - Timeout Value
 */
#define LPUART_TIMEOUT_TIMEOUT(x) \
    (((uint32_t)(((uint32_t)(x)) << LPUART_TIMEOUT_TIMEOUT_SHIFT)) & LPUART_TIMEOUT_TIMEOUT_MASK)

#define LPUART_TIMEOUT_CFG_MASK  (0xC0000000U)
#define LPUART_TIMEOUT_CFG_SHIFT (30U)
/*! CFG - Idle Configuration
 *  0b00..Flag sets after TIMEOUT characters are received
 *  0b01..Flag sets when idle for TIMEOUT bit clocks
 *  0b10..Flag sets when idle for TIMEOUT bit clocks following the next character
 *  0b11..Flag sets when idle for at least TIMEOUT bit clocks, but new character is detected before extended idle
 * timeout is reached
 */
#define LPUART_TIMEOUT_CFG(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TIMEOUT_CFG_SHIFT)) & LPUART_TIMEOUT_CFG_MASK)
/*! @} */

/* The count of LPUART_TIMEOUT */
#define LPUART_TIMEOUT_COUNT (4U)

/*! @name TCBR - Transmit Command Burst Register */
/*! @{ */

#define LPUART_TCBR_DATA_MASK  (0xFFFFU)
#define LPUART_TCBR_DATA_SHIFT (0U)
/*! DATA - Data
 */
#define LPUART_TCBR_DATA(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TCBR_DATA_SHIFT)) & LPUART_TCBR_DATA_MASK)
/*! @} */

/* The count of LPUART_TCBR */
#define LPUART_TCBR_COUNT (128U)

/*! @name TDBR - Transmit Data Burst Register */
/*! @{ */

#define LPUART_TDBR_DATA0_MASK  (0xFFU)
#define LPUART_TDBR_DATA0_SHIFT (0U)
/*! DATA0 - Data
 */
#define LPUART_TDBR_DATA0(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TDBR_DATA0_SHIFT)) & LPUART_TDBR_DATA0_MASK)

#define LPUART_TDBR_DATA1_MASK  (0xFF00U)
#define LPUART_TDBR_DATA1_SHIFT (8U)
/*! DATA1 - Data
 */
#define LPUART_TDBR_DATA1(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TDBR_DATA1_SHIFT)) & LPUART_TDBR_DATA1_MASK)

#define LPUART_TDBR_DATA2_MASK  (0xFF0000U)
#define LPUART_TDBR_DATA2_SHIFT (16U)
/*! DATA2 - Data
 */
#define LPUART_TDBR_DATA2(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TDBR_DATA2_SHIFT)) & LPUART_TDBR_DATA2_MASK)

#define LPUART_TDBR_DATA3_MASK  (0xFF000000U)
#define LPUART_TDBR_DATA3_SHIFT (24U)
/*! DATA3 - Data
 */
#define LPUART_TDBR_DATA3(x) (((uint32_t)(((uint32_t)(x)) << LPUART_TDBR_DATA3_SHIFT)) & LPUART_TDBR_DATA3_MASK)
/*! @} */

/* The count of LPUART_TDBR */
#define LPUART_TDBR_COUNT (256U)

/*!
 * @}
 */ /* end of group LPUART_Register_Masks */

/* LPUART - Peripheral instance base addresses */
#if (defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE & 0x2))
/** Peripheral LPUART1 base address */
#define LPUART1_BASE (0x54380000u)
/** Peripheral LPUART1 base address */
#define LPUART1_BASE_NS (0x44380000u)
/** Peripheral LPUART1 base pointer */
#define LPUART1 ((LPUART_Type *)LPUART1_BASE)
/** Peripheral LPUART1 base pointer */
#define LPUART1_NS ((LPUART_Type *)LPUART1_BASE_NS)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE (0x54390000u)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE_NS (0x44390000u)
/** Peripheral LPUART2 base pointer */
#define LPUART2 ((LPUART_Type *)LPUART2_BASE)
/** Peripheral LPUART2 base pointer */
#define LPUART2_NS ((LPUART_Type *)LPUART2_BASE_NS)
/** Peripheral LPUART3 base address */
#define LPUART3_BASE (0x52570000u)
/** Peripheral LPUART3 base address */
#define LPUART3_BASE_NS (0x42570000u)
/** Peripheral LPUART3 base pointer */
#define LPUART3 ((LPUART_Type *)LPUART3_BASE)
/** Peripheral LPUART3 base pointer */
#define LPUART3_NS ((LPUART_Type *)LPUART3_BASE_NS)
/** Peripheral LPUART4 base address */
#define LPUART4_BASE (0x52580000u)
/** Peripheral LPUART4 base address */
#define LPUART4_BASE_NS (0x42580000u)
/** Peripheral LPUART4 base pointer */
#define LPUART4 ((LPUART_Type *)LPUART4_BASE)
/** Peripheral LPUART4 base pointer */
#define LPUART4_NS ((LPUART_Type *)LPUART4_BASE_NS)
/** Peripheral LPUART5 base address */
#define LPUART5_BASE (0x52590000u)
/** Peripheral LPUART5 base address */
#define LPUART5_BASE_NS (0x42590000u)
/** Peripheral LPUART5 base pointer */
#define LPUART5 ((LPUART_Type *)LPUART5_BASE)
/** Peripheral LPUART5 base pointer */
#define LPUART5_NS ((LPUART_Type *)LPUART5_BASE_NS)
/** Peripheral LPUART6 base address */
#define LPUART6_BASE (0x525A0000u)
/** Peripheral LPUART6 base address */
#define LPUART6_BASE_NS (0x425A0000u)
/** Peripheral LPUART6 base pointer */
#define LPUART6 ((LPUART_Type *)LPUART6_BASE)
/** Peripheral LPUART6 base pointer */
#define LPUART6_NS ((LPUART_Type *)LPUART6_BASE_NS)
/** Peripheral LPUART7 base address */
#define LPUART7_BASE (0x52690000u)
/** Peripheral LPUART7 base address */
#define LPUART7_BASE_NS (0x42690000u)
/** Peripheral LPUART7 base pointer */
#define LPUART7 ((LPUART_Type *)LPUART7_BASE)
/** Peripheral LPUART7 base pointer */
#define LPUART7_NS ((LPUART_Type *)LPUART7_BASE_NS)
/** Peripheral LPUART8 base address */
#define LPUART8_BASE (0x526A0000u)
/** Peripheral LPUART8 base address */
#define LPUART8_BASE_NS (0x426A0000u)
/** Peripheral LPUART8 base pointer */
#define LPUART8 ((LPUART_Type *)LPUART8_BASE)
/** Peripheral LPUART8 base pointer */
#define LPUART8_NS ((LPUART_Type *)LPUART8_BASE_NS)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                                                                                     \
    {                                                                                                         \
        0u, LPUART1_BASE, LPUART2_BASE, LPUART3_BASE, LPUART4_BASE, LPUART5_BASE, LPUART6_BASE, LPUART7_BASE, \
            LPUART8_BASE                                                                                      \
    }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                                                                          \
    {                                                                                             \
        (LPUART_Type *)0u, LPUART1, LPUART2, LPUART3, LPUART4, LPUART5, LPUART6, LPUART7, LPUART8 \
    }
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS_NS                                                                                      \
    {                                                                                                             \
        0u, LPUART1_BASE_NS, LPUART2_BASE_NS, LPUART3_BASE_NS, LPUART4_BASE_NS, LPUART5_BASE_NS, LPUART6_BASE_NS, \
            LPUART7_BASE_NS, LPUART8_BASE_NS                                                                      \
    }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS_NS                                                                                    \
    {                                                                                                          \
        (LPUART_Type *)0u, LPUART1_NS, LPUART2_NS, LPUART3_NS, LPUART4_NS, LPUART5_NS, LPUART6_NS, LPUART7_NS, \
            LPUART8_NS                                                                                         \
    }
#else
/** Peripheral LPUART1 base address */
#define LPUART1_BASE (0x44380000u)
/** Peripheral LPUART1 base pointer */
#define LPUART1      ((LPUART_Type *)LPUART1_BASE)
/** Peripheral LPUART2 base address */
#define LPUART2_BASE (0x44390000u)
/** Peripheral LPUART2 base pointer */
#define LPUART2      ((LPUART_Type *)LPUART2_BASE)
/** Peripheral LPUART3 base address */
#define LPUART3_BASE (0x42570000u)
/** Peripheral LPUART3 base pointer */
#define LPUART3      ((LPUART_Type *)LPUART3_BASE)
/** Peripheral LPUART4 base address */
#define LPUART4_BASE (0x42580000u)
/** Peripheral LPUART4 base pointer */
#define LPUART4      ((LPUART_Type *)LPUART4_BASE)
/** Peripheral LPUART5 base address */
#define LPUART5_BASE (0x42590000u)
/** Peripheral LPUART5 base pointer */
#define LPUART5      ((LPUART_Type *)LPUART5_BASE)
/** Peripheral LPUART6 base address */
#define LPUART6_BASE (0x425A0000u)
/** Peripheral LPUART6 base pointer */
#define LPUART6      ((LPUART_Type *)LPUART6_BASE)
/** Peripheral LPUART7 base address */
#define LPUART7_BASE (0x42690000u)
/** Peripheral LPUART7 base pointer */
#define LPUART7      ((LPUART_Type *)LPUART7_BASE)
/** Peripheral LPUART8 base address */
#define LPUART8_BASE (0x426A0000u)
/** Peripheral LPUART8 base pointer */
#define LPUART8      ((LPUART_Type *)LPUART8_BASE)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                                                                                     \
    {                                                                                                         \
        0u, LPUART1_BASE, LPUART2_BASE, LPUART3_BASE, LPUART4_BASE, LPUART5_BASE, LPUART6_BASE, LPUART7_BASE, \
            LPUART8_BASE                                                                                      \
    }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                                                                          \
    {                                                                                             \
        (LPUART_Type *)0u, LPUART1, LPUART2, LPUART3, LPUART4, LPUART5, LPUART6, LPUART7, LPUART8 \
    }
#endif
/** Interrupt vectors for the LPUART peripheral type */
#define LPUART_RX_TX_IRQS                                                                                  \
    {                                                                                                      \
        NotAvail_IRQn, LPUART1_IRQn, LPUART2_IRQn, LPUART3_IRQn, LPUART4_IRQn, LPUART5_IRQn, LPUART6_IRQn, \
            LPUART7_IRQn, LPUART8_IRQn                                                                     \
    }

/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */

#endif  /* #if !defined(MIMX95_LPUART_H_) */
