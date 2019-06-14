/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2013 OpenMusicKontrollers.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file libmaple/stm32f3/include/series/usart.h
 * @author F3-port by Hanspeter Portner <dev@open-music-kontrollers.ch>
 * @brief STM32F3 USART support.
 */

#ifndef _LIBMAPLE_STM32F3_USART_H_
#define _LIBMAPLE_STM32F3_USART_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/gpio.h>      /* for gpio_af */
#include <libmaple/libmaple_types.h>
#include <libmaple/util.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
#include <libmaple/ring_buffer.h>

/*
 * Register maps
 */

/** USART register map type */
typedef struct usart_reg_map {
    __IO uint32 CR1;            /**< Control register 1 */
    __IO uint32 CR2;            /**< Control register 2 */
    __IO uint32 CR3;            /**< Control register 3 */
    __IO uint32 BRR;            /**< Baud rate register */
    __IO uint32 GTPR;           /**< Guard time and prescaler register */
    __IO uint32 RTOR;           /**< Receiver timeout register */
    __IO uint32 RQR;            /**< Request register */
    __IO uint32 SR;             /**< ISR Interrupt and status register */
    __IO uint32 ICR;            /**< Interrupt clear register */
    __IO uint16 RDR;            /**< Receive data register */
		uint16 RESERVED1;
    __IO uint16 TDR;            /**< Transmit data register */
		uint16 RESERVED2;
} usart_reg_map;

/*
 * Register map base pointers
 */

/** USART1 register map base pointer */
#define USART1_BASE                     ((struct usart_reg_map*)0x40013800)
/** USART2 register map base pointer */
#define USART2_BASE                     ((struct usart_reg_map*)0x40004400)
/** USART3 register map base pointer */
#define USART3_BASE                     ((struct usart_reg_map*)0x40004800)

#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
/** UART4 register map base pointer */
#define UART4_BASE                      ((struct usart_reg_map*)0x40004C00)
/** UART5 register map base pointer */
#define UART5_BASE                      ((struct usart_reg_map*)0x40005000)
#endif


/* Guard time and prescaler register */

/**
 * @brief Guard time value mask
 * Used in Smartcard mode. Not available on UART4, UART5. */
#define USART_GTPR_GT                   (0xFF << 8)
/**
 * @brief Prescaler value mask
 * Restrictions on this value apply, depending on the USART mode. Not
 * available on UART4, UART5. */
#define USART_GTPR_PSC                  0xFF

/*
 * Devices
 */

#ifndef USART_RX_BUF_SIZE
#define USART_RX_BUF_SIZE               64
#endif


/*
 * Devices
 */
/** USART device type */
typedef struct usart_dev {
    usart_reg_map *regs;             /**< Register map */
    ring_buffer *rb;                 /**< RX ring buffer */
    uint32 max_baud;                 /**< @brief Deprecated.
                                      * Maximum baud rate. */
    uint8 rx_buf[USART_RX_BUF_SIZE]; /**< @brief Deprecated.
                                      * Actual RX buffer used by rb.
                                      * This field will be removed in
                                      * a future release. */
    rcc_clk_id clk_id;               /**< RCC clock information */
    nvic_irq_num irq_num;            /**< USART NVIC interrupt */
} usart_dev;


struct usart_dev;
extern struct usart_dev *USART1;
extern struct usart_dev *USART2;
extern struct usart_dev *USART3;
#ifdef STM32_HIGH_DENSITY
extern struct usart_dev *UART4;
extern struct usart_dev *UART5;
#endif

/*
 * F3-only register bit definitions.
 */

/* Control register 1 */
#define USART_CR1_EOBIE_BIT					27
#define USART_CR1_RTOIE_BIT					26
#define USART_CR1_DEAT4_BIT					25
#define USART_CR1_DEAT3_BIT					24
#define USART_CR1_DEAT2_BIT					23
#define USART_CR1_DEAT1_BIT					22
#define USART_CR1_DEAT0_BIT					21
#define USART_CR1_DEDT4_BIT					20
#define USART_CR1_DEDT3_BIT					19
#define USART_CR1_DEDT2_BIT					18
#define USART_CR1_DEDT1_BIT					17
#define USART_CR1_DEDT0_BIT					16
#define USART_CR1_OVER8_BIT					15
#define USART_CR1_CMIE_BIT					14
#define USART_CR1_MME_BIT						13
#define USART_CR1_UESM_BIT					1
#define USART_CR1_UE_BIT						0

#define USART_CR1_EOBIE							(1UL << USART_CR1_EOBIE_BIT)
#define USART_CR1_RTOIE             (1UL << USART_CR1_RTOIE_BIT)
#define USART_CR1_DEAT4             (1UL << USART_CR1_DEAT4_BIT)
#define USART_CR1_DEAT3             (1UL << USART_CR1_DEAT3_BIT)
#define USART_CR1_DEAT2             (1UL << USART_CR1_DEAT2_BIT)
#define USART_CR1_DEAT1             (1UL << USART_CR1_DEAT1_BIT)
#define USART_CR1_DEAT0             (1UL << USART_CR1_DEAT0_BIT)
#define USART_CR1_DEDT4             (1UL << USART_CR1_DEDT4_BIT)
#define USART_CR1_DEDT3             (1UL << USART_CR1_DEDT3_BIT)
#define USART_CR1_DEDT2             (1UL << USART_CR1_DEDT2_BIT)
#define USART_CR1_DEDT1             (1UL << USART_CR1_DEDT1_BIT)
#define USART_CR1_DEDT0             (1UL << USART_CR1_DEDT0_BIT)
#define USART_CR1_OVER8             (1UL << USART_CR1_OVER8_BIT)
#define USART_CR1_CMIE              (1UL << USART_CR1_CMIE_BIT)
#define USART_CR1_MME               (1UL << USART_CR1_MME_BIT)
#define USART_CR1_UESM              (1UL << USART_CR1_UESM_BIT)
#define USART_CR1_UE                (1UL << USART_CR1_UE_BIT)

/* Control register 2 */
#define USART_CR2_ADD_SHIFT					24
#define USART_CR2_RTOEN_BIT					23
#define USART_CR2_ABRMOD1_BIT				22
#define USART_CR2_ABRMOD0_BIT				21
#define USART_CR2_ABREN_BIT					20
#define USART_CR2_MSBFIRST_BIT			19
#define USART_CR2_DATAINV_BIT				18
#define USART_CR2_TXINV_BIT					17
#define USART_CR2_RXINV_BIT					16
#define USART_CR2_SWAP_BIT					15
#define USART_CR2_ADDM7_BIT					4

#define USART_CR2_ADD								(0xFF << USART_CR2_ADD_SHIFT)
#define USART_CR2_RTOEN							(1UL << USART_CR2_RTOEN_BIT)
#define USART_CR2_ABRMOD1           (1UL << USART_CR2_ABRMOD1_BIT)
#define USART_CR2_ABRMOD0           (1UL << USART_CR2_ABRMOD0_BIT)
#define USART_CR2_ABREN             (1UL << USART_CR2_ABREN_BIT)
#define USART_CR2_MSBFIRST          (1UL << USART_CR2_MSBFIRST_BIT)
#define USART_CR2_DATAINV           (1UL << USART_CR2_DATAINV_BIT)
#define USART_CR2_TXINV             (1UL << USART_CR2_TXINV_BIT)
#define USART_CR2_RXINV             (1UL << USART_CR2_RXINV_BIT)
#define USART_CR2_SWAP              (1UL << USART_CR2_SWAP_BIT)
#define USART_CR2_ADDM7             (1UL << USART_CR2_ADDM7_BIT)

/* Control register 3 */
#define USART_CR3_WUFIE_BIT					22
#define USART_CR3_WUS_SHIFT					20
#define USART_CR3_SCAR_SHIFT				17
#define USART_CR3_DEP_BIT						15
#define USART_CR3_DEM_BIT						14
#define USART_CR3_DDRE_BIT					13
#define USART_CR3_OVRDIS_BIT				12
#define USART_CR3_ONEBIT_BIT				11

#define USART_CR3_WUFIE							(1UL << USART_CR3_WUFIE_BIT)
#define USART_CR3_WUS								(0x3 << USART_CR3_WUS_SHIFT)
#define USART_CR3_SCAR							(0x7 << USART_CR3_SCAR_SHIFT)
#define USART_CR3_DEP								(1UL << USART_CR3_DEP_BIT)
#define USART_CR3_DEM               (1UL << USART_CR3_DEM_BIT)
#define USART_CR3_DDRE              (1UL << USART_CR3_DDRE_BIT)
#define USART_CR3_OVRDIS            (1UL << USART_CR3_OVRDIS_BIT)
#define USART_CR3_ONEBIT            (1UL << USART_CR3_ONEBIT_BIT)

/* Receive timeout register */
#define USART_RTOR_BLEN_SHIFT				24
#define USART_RTOR_RTO_SHIFT				0

#define USART_RTOR_BLEN							(0xF << USART_RTOR_BLEN_SHIFT)
#define USART_RTOR_RTO							(0xFFF << USART_RTOR_RTO_SHIFT)

/* Request register */
#define USART_RQR_TXFRQ_BIT					4
#define USART_RQR_RXFRQ_BIT					3
#define USART_RQR_MMRQ_BIT					2
#define USART_RQR_SBKRQ_BIT					1
#define USART_RQR_ABRRQ_BIT					0

#define USART_RQR_TXFRQ							(1UL << USART_RQR_TXFRQ_BIT)
#define USART_RQR_RXFRQ             (1UL << USART_RQR_RXFRQ_BIT)
#define USART_RQR_MMRQ              (1UL << USART_RQR_MMRQ_BIT)
#define USART_RQR_SBKRQ             (1UL << USART_RQR_SBKRQ_BIT)
#define USART_RQR_ABRRQ             (1UL << USART_RQR_ABRRQ_BIT)

/* Interrupt and status register */
// common register bits with other STM32 series are defined as USART_SR_* for compatibility 
#define USART_SR_REACK_BIT					22
#define USART_SR_TEACK_BIT					21
#define USART_SR_WUF_BIT						20
#define USART_SR_RWU_BIT						19
#define USART_SR_SBKF_BIT						18
#define USART_SR_CMF_BIT						17
#define USART_SR_BUSY_BIT						16
#define USART_SR_ABRF_BIT						15
#define USART_SR_ABRE_BIT						14
#define USART_SR_EOBF_BIT						12
#define USART_SR_RTOF_BIT						11
#define USART_SR_CTS_BIT						10
#define USART_SR_CTSIF_BIT					9

#define USART_SR_REACK							(1UL << USART_ISR_REACK_BIT)
#define USART_SR_TEACK        	    (1UL << USART_ISR_TEACK_BIT)
#define USART_SR_WUF          	    (1UL << USART_ISR_WUF_BIT)
#define USART_SR_RWU          	    (1UL << USART_ISR_RWU_BIT)
#define USART_SR_SBKF         	    (1UL << USART_ISR_SBKF_BIT)
#define USART_SR_CMF          	    (1UL << USART_ISR_CMF_BIT)
#define USART_SR_BUSY         	    (1UL << USART_ISR_BUSY_BIT)
#define USART_SR_ABRF         	    (1UL << USART_ISR_ABRF_BIT)
#define USART_SR_ABRE         	    (1UL << USART_ISR_ABRE_BIT)
#define USART_SR_EOBF         	    (1UL << USART_ISR_EOBF_BIT)
#define USART_SR_RTOF         	    (1UL << USART_ISR_RTOF_BIT)
#define USART_SR_CTS	        	    (1UL << USART_ISR_CTS_BIT)
#define USART_SR_CTSIF        	    (1UL << USART_ISR_CTSIF_BIT)

/* Interrupt clear register */
#define USART_ICR_WUFCF_BIT					20
#define USART_ICR_CMCF_BIT					17
#define USART_ICR_EOBCF_BIT					12
#define USART_ICR_RTOCF_BIT					11
#define USART_ICR_CTSCF_BIT					9
#define USART_ICR_LBDCF_BIT					8
#define USART_ICR_TCCF_BIT					6
#define USART_ICR_IDLECF_BIT				4
#define USART_ICR_ORECF_BIT					3
#define USART_ICR_NCF_BIT						2
#define USART_ICR_FECF_BIT					1
#define USART_ICR_PECF_BIT					0

#define USART_ICR_WUFCF        			(1UL << USART_ICR_WUFCF_BIT)
#define USART_ICR_CMCF         			(1UL << USART_ICR_CMCF_BIT)
#define USART_ICR_EOBCF        			(1UL << USART_ICR_EOBCF_BIT)
#define USART_ICR_RTOCF        			(1UL << USART_ICR_RTOCF_BIT)
#define USART_ICR_CTSCF        			(1UL << USART_ICR_CTSCF_BIT)
#define USART_ICR_LBDCF        			(1UL << USART_ICR_LBDCF_BIT)
#define USART_ICR_TCCF         			(1UL << USART_ICR_TCCF_BIT)
#define USART_ICR_IDLECF       			(1UL << USART_ICR_IDLECF_BIT)
#define USART_ICR_ORECF        			(1UL << USART_ICR_ORECF_BIT)
#define USART_ICR_NCF          			(1UL << USART_ICR_NCF_BIT)
#define USART_ICR_FECF         			(1UL << USART_ICR_FECF_BIT)
#define USART_ICR_PECF         			(1UL << USART_ICR_PECF_BIT)

/* Receive data register */
#define USART_RDR_RDR_SHIFT					0

#define USART_RDR_RDR								(0x1FF << USART_RDR_RDR_SHIFT)


/* Transmit data register */
#define USART_TDR_TDR_SHIFT					0

#define USART_TDR_TDR								(0x1FF << USART_TDR_TDR_SHIFT)


/*
 * Register bit definitions
 */

/* Status register */

/** Line break detection bit */
#define USART_SR_LBD_BIT                8
/** Transmit data register empty bit */
#define USART_SR_TXE_BIT                7
/** Transmission complete bit */
#define USART_SR_TC_BIT                 6
/** Read data register not empty bit */
#define USART_SR_RXNE_BIT               5
/** IDLE line detected bit */
#define USART_SR_IDLE_BIT               4
/** Overrun error bit */
#define USART_SR_ORE_BIT                3
/** Noise error bit */
#define USART_SR_NE_BIT                 2
/**
 * @brief Synonym for USART_SR_NE_BIT.
 *
 * Some series (e.g. STM32F2) use "NF" for "noise flag" instead of the
 * original "NE" for "noise error". The meaning of the bit is
 * unchanged, but the NF flag can be disabled when the line is
 * noise-free.
 *
 * @see USART_SR_NE_BIT
 */
#define USART_SR_NF_BIT                 USART_SR_NE_BIT
/** Framing error bit */
#define USART_SR_FE_BIT                 1
/** Parity error bit */
#define USART_SR_PE_BIT                 0

/** Line break detected mask */
#define USART_SR_LBD                    BIT(USART_SR_LBD_BIT)
/** Transmit data register empty mask */
#define USART_SR_TXE                    BIT(USART_SR_TXE_BIT)
/** Transmission complete mask */
#define USART_SR_TC                     BIT(USART_SR_TC_BIT)
/** Read data register not empty mask */
#define USART_SR_RXNE                   BIT(USART_SR_RXNE_BIT)
/** IDLE line detected mask */
#define USART_SR_IDLE                   BIT(USART_SR_IDLE_BIT)
/** Overrun error mask */
#define USART_SR_ORE                    BIT(USART_SR_ORE_BIT)
/** Noise error mask */
#define USART_SR_NE                     BIT(USART_SR_NE_BIT)
/**
 * @brief Synonym for USART_SR_NE.
 * @see USART_SR_NF_BIT
 */
#define USART_SR_NF                     USART_SR_NE
/** Framing error mask */
#define USART_SR_FE                     BIT(USART_SR_FE_BIT)
/** Parity error mask */
#define USART_SR_PE                     BIT(USART_SR_PE_BIT)

/* Data register */

/** Data register data value mask */
#define USART_DR_DR                     0xFF

/* Baud rate register */

/** Mantissa of USARTDIV mask */
#define USART_BRR_DIV_MANTISSA          (0xFFF << 4)
/** Fraction of USARTDIV mask */
#define USART_BRR_DIV_FRACTION          0xF

/* Control register 1 */

/** Word length bit */
#define USART_CR1_M_BIT                 12
/** Wakeup method bit */
#define USART_CR1_WAKE_BIT              11
/** Parity control enable bit */
#define USART_CR1_PCE_BIT               10
/** Parity selection bit */
#define USART_CR1_PS_BIT                9
/** Parity error interrupt enable bit */
#define USART_CR1_PEIE_BIT              8
/** Transmit data regsiter not empty interrupt enable bit */
#define USART_CR1_TXEIE_BIT             7
/** Transmission complete interrupt enable bit */
#define USART_CR1_TCIE_BIT              6
/** RXNE interrupt enable bit */
#define USART_CR1_RXNEIE_BIT            5
/** IDLE interrupt enable bit */
#define USART_CR1_IDLEIE_BIT            4
/** Transmitter enable bit */
#define USART_CR1_TE_BIT                3
/** Receiver enable bit */
#define USART_CR1_RE_BIT                2

/** Word length mask */
#define USART_CR1_M                     BIT(USART_CR1_M_BIT)
/** Word length: 1 start bit, 8 data bits, n stop bit */
#define USART_CR1_M_8N1                 (0 << USART_CR1_M_BIT)
/** Word length: 1 start bit, 9 data bits, n stop bit */
#define USART_CR1_M_9N1                 (1 << USART_CR1_M_BIT)
/** Wakeup method mask */
#define USART_CR1_WAKE                  BIT(USART_CR1_WAKE_BIT)
/** Wakeup on idle line */
#define USART_CR1_WAKE_IDLE             (0 << USART_CR1_WAKE_BIT)
/** Wakeup on address mark */
#define USART_CR1_WAKE_ADDR             (1 << USART_CR1_WAKE_BIT)
/** Parity control enable mask */
#define USART_CR1_PCE                   BIT(USART_CR1_PCE_BIT)
/** Parity selection mask */
#define USART_CR1_PS                    BIT(USART_CR1_PS_BIT)
/** Parity selection: even parity */
#define USART_CR1_PS_EVEN               (0 << USART_CR1_PS_BIT)
/** Parity selection: odd parity */
#define USART_CR1_PS_ODD                (1 << USART_CR1_PS_BIT)
/** Parity error interrupt enable mask */
#define USART_CR1_PEIE                  BIT(USART_CR1_PEIE_BIT)
/** Transmit data register empty interrupt enable mask */
#define USART_CR1_TXEIE                 BIT(USART_CR1_TXEIE_BIT)
/** Transmission complete interrupt enable mask */
#define USART_CR1_TCIE                  BIT(USART_CR1_TCIE_BIT)
/** RXNE interrupt enable mask */
#define USART_CR1_RXNEIE                BIT(USART_CR1_RXNEIE_BIT)
/** IDLE line interrupt enable mask */
#define USART_CR1_IDLEIE                BIT(USART_CR1_IDLEIE_BIT)
/** Transmitter enable mask */
#define USART_CR1_TE                    BIT(USART_CR1_TE_BIT)
/** Receiver enable mask */
#define USART_CR1_RE                    BIT(USART_CR1_RE_BIT)

/* Control register 2 */

/** LIN mode enable bit */
#define USART_CR2_LINEN_BIT             14
/** Clock enable bit */
#define USART_CR2_CLKEN_BIT             11
/** Clock polarity bit */
#define USART_CR2_CPOL_BIT              10
/** Clock phase bit */
#define USART_CR2_CPHA_BIT              9
/** Last bit clock pulse bit */
#define USART_CR2_LBCL_BIT              8
/** LIN break detection interrupt enable bit */
#define USART_CR2_LBDIE_BIT             6
/** LIN break detection length bit */
#define USART_CR2_LBDL_BIT              5

/** LIN mode enable mask */
#define USART_CR2_LINEN                 BIT(USART_CR2_LINEN_BIT)
/** STOP bits mask */
#define USART_CR2_STOP                  (0x3 << 12)
/** STOP bits: 1 stop bit */
#define USART_CR2_STOP_BITS_1           (0x0 << 12)
/**
 * @brief STOP bits: 0.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_POINT_5     (0x1 << 12)
/** STOP bits: 2 stop bits */
#define USART_CR2_STOP_BITS_2           (0x2 << 12)
/**
 * @brief STOP bits: 1.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_1_POINT_5   (0x3 << 12)
/**
 * @brief Clock enable.
 * Not available on UART4, UART5 */
#define USART_CR2_CLKEN                 BIT(USART_CR2_CLKEN_BIT)
/**
 * @brief Clock polarity mask.
 * Not available on UART4, UART5 */
#define USART_CR2_CPOL                  BIT(USART_CR2_CPOL_BIT)
/** Clock polarity: low */
#define USART_CR2_CPOL_LOW              (0x0 << USART_CR2_CLKEN_BIT)
/** Clock polarity: high */
#define USART_CR2_CPOL_HIGH             (0x1 << USART_CR2_CLKEN_BIT)
/**
 * @brief Clock phase mask.
 * Not available on UART4, UART5 */
#define USART_CR2_CPHA                  BIT(USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: first
 * First clock transition is the first data capture edge. */
#define USART_CR2_CPHA_FIRST            (0x0 << USART_CR2_CPHA_BIT)
/**
 * @brief Clock phase: second
 * Second clock transition is the first data capture edge. */
#define USART_CR2_CPHA_SECOND           (0x1 << USART_CR2_CPHA_BIT)
/**
 * @brief Last bit clock pulse mask.
 *
 * When set, the last bit transmitted causes a clock pulse in
 * synchronous mode.
 *
 * Not available on UART4, UART5 */
#define USART_CR2_LBCL                  BIT(USART_CR2_LBCL_BIT)
/** LIN break detection interrupt enable mask. */
#define USART_CR2_LBDIE                 BIT(USART_CR2_LBDIE_BIT)
/** LIN break detection length. */
#define USART_CR2_LBDL                  BIT(USART_CR2_LBDL_BIT)
/** LIN break detection length: 10 bits */
#define USART_CR2_LBDL_10_BIT           (0 << USART_CR2_LBDL_BIT)
/** LIN break detection length: 11 bits */
#define USART_CR2_LBDL_11_BIT           (1 << USART_CR2_LBDL_BIT)

/* Control register 3 */

/** Clear to send interrupt enable bit */
#define USART_CR3_CTSIE_BIT             10
/** Clear to send enable bit */
#define USART_CR3_CTSE_BIT              9
/** Ready to send enable bit */
#define USART_CR3_RTSE_BIT              8
/** DMA enable transmitter bit */
#define USART_CR3_DMAT_BIT              7
/** DMA enable receiver bit */
#define USART_CR3_DMAR_BIT              6
/** Smartcard mode enable bit */
#define USART_CR3_SCEN_BIT              5
/** Smartcard NACK enable bit */
#define USART_CR3_NACK_BIT              4
/** Half-duplex selection bit */
#define USART_CR3_HDSEL_BIT             3
/** IrDA low power bit */
#define USART_CR3_IRLP_BIT              2
/** IrDA mode enable bit */
#define USART_CR3_IREN_BIT              1
/** Error interrupt enable bit */
#define USART_CR3_EIE_BIT               0

/**
 * @brief Clear to send interrupt enable
 * Not available on UART4, UART5. */
#define USART_CR3_CTSIE                 BIT(USART_CR3_CTSIE_BIT)
/**
 * @brief Clear to send enable
 * Not available on UART4, UART5. */
#define USART_CR3_CTSE                  BIT(USART_CR3_CTSE_BIT)
/**
 * @brief Ready to send enable
 * Not available on UART4, UART5. */
#define USART_CR3_RTSE                  BIT(USART_CR3_RTSE_BIT)
/**
 * @brief DMA enable transmitter
 * Not available on UART5. */
#define USART_CR3_DMAT                  BIT(USART_CR3_DMAT_BIT)
/**
 * @brief DMA enable receiver
 * Not available on UART5. */
#define USART_CR3_DMAR                  BIT(USART_CR3_DMAR_BIT)
/**
 * @brief Smartcard mode enable
 * Not available on UART4, UART5. */
#define USART_CR3_SCEN                  BIT(USART_CR3_SCEN_BIT)
/**
 * @brief Smartcard NACK enable
 * Not available on UART4, UART5. */
#define USART_CR3_NACK                  BIT(USART_CR3_NACK_BIT)
/**
 * @brief Half-duplex selection
 * When set, single-wire half duplex mode is selected.
 */
#define USART_CR3_HDSEL                 BIT(USART_CR3_HDSEL_BIT)
/** IrDA low power mode */
#define USART_CR3_IRLP                  BIT(USART_CR3_IRLP_BIT)
/** IrDA mode: normal */
#define USART_CR3_IRLP_NORMAL           (0U << USART_CR3_IRLP_BIT)
/** IrDA mode: low power */
#define USART_CR3_IRLP_LOW_POWER        (1U << USART_CR3_IRLP_BIT)
/** IrDA mode enable */
#define USART_CR3_IREN                  BIT(USART_CR3_IREN_BIT)
/** Error interrupt enable */
#define USART_CR3_EIE                   BIT(USART_CR3_EIE_BIT)

/* Guard time and prescaler register */

/**
 * @brief Guard time value mask
 * Used in Smartcard mode. Not available on UART4, UART5. */
#define USART_GTPR_GT                   (0xFF << 8)
/**
 * @brief Prescaler value mask
 * Restrictions on this value apply, depending on the USART mode. Not
 * available on UART4, UART5. */
#define USART_GTPR_PSC                  0xFF

/*
 * Devices
 */

#ifndef USART_RX_BUF_SIZE
#define USART_RX_BUF_SIZE               64
#endif

void usart_init(usart_dev *dev);

struct gpio_dev;                /* forward declaration */
/* FIXME [PRE 0.0.13] decide if flags are necessary */
/**
 * @brief Configure GPIOs for use as USART TX/RX.
 * @param udev USART device to use
 * @param rx_dev RX pin gpio_dev
 * @param rx     RX pin bit on rx_dev
 * @param tx_dev TX pin gpio_dev
 * @param tx     TX pin bit on tx_dev
 * @param flags  Currently ignored
 */
extern void usart_config_gpios_async(usart_dev *udev,
                                     struct gpio_dev *rx_dev, uint8 rx,
                                     struct gpio_dev *tx_dev, uint8 tx,
                                     unsigned flags);

#define USART_USE_PCLK 0
void usart_set_baud_rate(usart_dev *dev, uint32 clock_speed, uint32 baud);

void usart_enable(usart_dev *dev);
void usart_disable(usart_dev *dev);
void usart_foreach(void (*fn)(usart_dev *dev));
/**
 * @brief Nonblocking USART transmit
 * @param dev Serial port to transmit over
 * @param buf Buffer to transmit
 * @param len Maximum number of bytes to transmit
 * @return Number of bytes transmitted
 */
uint32 usart_tx(usart_dev *dev, const uint8 *buf, uint32 len);
uint32 usart_rx(usart_dev *dev, uint8 *buf, uint32 len);
void usart_putudec(usart_dev *dev, uint32 val);

/**
 * @brief Disable all serial ports.
 */
static inline void usart_disable_all(void) {
    usart_foreach(usart_disable);
}

/**
 * @brief Transmit one character on a serial port.
 *
 * This function blocks until the character has been successfully
 * transmitted.
 *
 * @param dev Serial port to send on.
 * @param byte Byte to transmit.
 */
static inline void usart_putc(usart_dev* dev, uint8 byte) {
    while (!usart_tx(dev, &byte, 1))
        ;
}

/**
 * @brief Transmit a character string on a serial port.
 *
 * This function blocks until str is completely transmitted.
 *
 * @param dev Serial port to send on
 * @param str String to send
 */
static inline void usart_putstr(usart_dev *dev, const char* str) {
    uint32 i = 0;
    while (str[i] != '\0') {
        usart_putc(dev, str[i++]);
    }
}

/**
 * @brief Read one character from a serial port.
 *
 * It's not safe to call this function if the serial port has no data
 * available.
 *
 * @param dev Serial port to read from
 * @return byte read
 * @see usart_data_available()
 */
static inline uint8 usart_getc(usart_dev *dev) {
    return rb_remove(dev->rb);
}

/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint32 usart_data_available(usart_dev *dev) {
    return rb_full_count(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(usart_dev *dev) {
    rb_reset(dev->rb);
}
/*
 * Routines
 */

gpio_af usart_get_af(struct usart_dev *dev);

#ifdef __cplusplus
}
#endif

#endif
