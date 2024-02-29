/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 * @file libmaple/include/libmaple/usart.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>,
 *         Perry Hung <perry@leaflabs.com>
 * @brief USART definitions and prototypes
 */

#ifndef _LIBMAPLE_USART_H_
#define _LIBMAPLE_USART_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "libmaple_types.h"
#include "util.h"
#include "rcc.h"
#include "nvic.h"
#include "usb/usb_cdc.h"

/*
 * Register map (common across supported STM32 series).
 */

/** USART register map type */
typedef struct usart_reg_map {
    __IO uint32 SR;             /**< Status register */
    __IO uint32 DR;             /**< Data register */
    __IO uint32 BRR;            /**< Baud rate register */
    __IO uint32 CR1;            /**< Control register 1 */
    __IO uint32 CR2;            /**< Control register 2 */
    __IO uint32 CR3;            /**< Control register 3 */
    __IO uint32 GTPR;           /**< Guard time and prescaler register */
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
#ifdef STM32_HIGH_DENSITY
/** UART4 register map base pointer */
#define UART4_BASE                      ((struct usart_reg_map*)0x40004C00)
/** UART5 register map base pointer */
#define UART5_BASE                      ((struct usart_reg_map*)0x40005000)
#endif



/*
 * Register bit definitions
 */

/* Status register */

/** Clear to send bit */
#define USART_SR_CTS_BIT                9
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

/** Clear to send mask */
#define USART_SR_CTS                    BIT(USART_SR_CTS_BIT)
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

/** USART enable bit */
#define USART_CR1_UE_BIT                13
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
/** Receiver wakeup bit */
#define USART_CR1_RWU_BIT               1
/** Send break bit */
#define USART_CR1_SBK_BIT               0

/** USART enable mask */
#define USART_CR1_UE                    BIT(USART_CR1_UE_BIT)
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
/** Receiver wakeup mask */
#define USART_CR1_RWU                   BIT(USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in active mode */
#define USART_CR1_RWU_ACTIVE            (0 << USART_CR1_RWU_BIT)
/** Receiver wakeup: receiver in mute mode */
#define USART_CR1_RWU_MUTE              (1 << USART_CR1_RWU_BIT)
/** Send break */
#define USART_CR1_SBK                   BIT(USART_CR1_SBK_BIT)

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
#define USART_CR2_STOP_MASK             (0x3 << 12)
/** STOP bits: 1 stop bit */
#define USART_CR2_STOP_BITS_1           (0x0 << 12)
/**
 * @brief STOP bits: 0.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_0_5         (0x1 << 12)
/** STOP bits: 2 stop bits */
#define USART_CR2_STOP_BITS_2           (0x2 << 12)
/**
 * @brief STOP bits: 1.5 stop bits
 * Not available  on UART4, UART5. */
#define USART_CR2_STOP_BITS_1_5         (0x3 << 12)
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
/**
 * @brief Address of the USART node
 * This is useful during multiprocessor communication. */
#define USART_CR2_ADD                   0xF

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

#ifndef USART_TX_BUF_SIZE
#define USART_TX_BUF_SIZE               64
#endif

#define SERIAL_8N1	0B00000000
#define SERIAL_8N2	0B00100000
#define SERIAL_9N1	0B00001000
#define SERIAL_9N2	0B00101000

#define SERIAL_8E1	0B00001010
#define SERIAL_8E2	0B00101010
/* not supported:
#define SERIAL_9E1	0B00001010
#define SERIAL_9E2	0B00101010
*/
#define SERIAL_8O1	0B00001011
#define SERIAL_8O2	0B00101011
/* not supported:
#define SERIAL_9O1	0B00001011
#define SERIAL_9O2	0B00101011
*/

#include "ring_buffer.h"

/** USART device type */
typedef struct {
    usart_reg_map * regs;             /**< Register map */
    ring_buffer_t * rb;               /**< RX ring buffer */
    ring_buffer_t * wb;               /**< TX ring buffer */
    rcc_clk_id clk_id;               /**< RCC clock information */
    nvic_irq_num irq_num;            /**< USART NVIC interrupt */
} usart_dev_t;

/*
 * Devices
 */
extern const usart_dev_t usart1;
extern const usart_dev_t usart2;
extern const usart_dev_t usart3;
#define USART1 (&usart1)
#define USART2 (&usart2)
#define USART3 (&usart3)

#ifdef STM32_HIGH_DENSITY
extern const usart_dev_t uart4;
extern const usart_dev_t uart5;
#define UART4 (&uart4)
#define UART5 (&uart5)
extern const usart_dev_t * const usart_devs[5];
#else
extern const usart_dev_t * const usart_devs[3];
#endif

enum uart_parity_t { UART_PARITY_EVEN = USART_CR1_PS_EVEN, UART_PARITY_ODD = USART_CR1_PS_ODD };
enum uart_stop_bits_t { UART_STOP_BITS_0_5 = USART_CR2_STOP_BITS_0_5, UART_STOP_BITS_1 = USART_CR2_STOP_BITS_1,
	UART_STOP_BITS_1_5 = USART_CR2_STOP_BITS_1_5, UART_STOP_BITS_2 = USART_CR2_STOP_BITS_2 };


// Define constants and variables for buffering incoming serial data.

#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 64


void usart_init(const usart_dev_t *dev);

/* FIXME [PRE 0.0.13] decide if flags are necessary */
/**
 * @brief Configure GPIOs for use as USART TX/RX.
 * @param udev USART device to use
 * @param rx_dev RX pin gpio_dev_t
 * @param rx     RX pin bit on rx_dev
 * @param tx_dev TX pin gpio_dev_t
 * @param tx     TX pin bit on tx_dev
 * @param flags  Currently ignored
 */

void usart_config(const usart_dev_t *udev, uint8 rx_pin, uint8 tx_pin, uint8_t flags);
void usart_config_line_coding(const usart_dev_t *udev, usb_cdcacm_line_coding_t * lc);
void usart_set_baud_rate(const usart_dev_t *dev, uint32 baud);

void usart_enable(const usart_dev_t *dev);
void usart_disable(const usart_dev_t *dev);
void usart_foreach(void(*fn)(const usart_dev_t *dev));
uint16 usart_tx(const usart_dev_t *dev, const uint8 *buf, uint16 n);
void usart_tx_fast(const usart_dev_t *dev, uint8 *buf, uint16 n);
uint16 usart_rx(const usart_dev_t *dev, uint8 *buf, uint16 n);
void usart_putudec(const usart_dev_t *dev, uint32 val);

/**
 * @brief Disable all serial ports.
 */
static inline void usart_disable_all(void) {
    usart_foreach(usart_disable);
}

/**
 * @brief Transmit one character on a serial port.
 *
 * This function blocks until the character has been queued
 * for transmission.
 *
 * @param dev Serial port to send on.
 * @param byte Byte to transmit.
 */
static inline void usart_putc(const usart_dev_t* dev, char byte) {
    while (!usart_tx(dev, (const uint8*)&byte, 1))
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
static inline void usart_putstr(const usart_dev_t *dev, const char* str) {
    char c;
    while ( (c=*str++) != '\0') {
        usart_putc(dev, c);
    }
}

/**
 * @brief Read one character from a serial port.
 *
 * It's not safe to call this function if the serial port has no data available
 * Only use in combination with: if ( usart_rx_available() > 0 )
 *
 * @param dev Serial port to read from
 * @return byte read
 * @see usart_data_available()
 */
static inline uint8_t usart_getc(const usart_dev_t *dev) {
    return rb_read(dev->rb);
}

/*
 * Roger Clark. 20141125,
 * added peek function.
 * @param dev Serial port to read from
 * @return byte read
 */
static inline int usart_peek(const usart_dev_t *dev) {
	return rb_peek(dev->rb);
}
/**
 * @brief Return the amount of free slots in a serial port's TX buffer.
 * @param dev Serial port to check
 * @return Number of free slots in dev's TX buffer.
 */
static inline uint16_t usart_tx_available(const usart_dev_t *dev) {
    return rb_write_available(dev->wb);
}
/*
static inline uint8_t * usart_tx_ptr(usart_dev_t *dev) {
    return rb_write_ptr(dev->wb);
}
*/
static inline void usart_tx_start(const usart_dev_t *dev) {
	dev->regs->CR1 |= USART_CR1_TXEIE;
}
/*
static inline void usart_tx_finish(usart_dev_t *dev, uint16_t nr) {
    rb_write_finish(dev->txB, nr);
}
*/
/**
 * @brief Return the amount of data available in a serial port's RX buffer.
 * @param dev Serial port to check
 * @return Number of bytes in dev's RX buffer.
 */
static inline uint16 usart_rx_available(const usart_dev_t *dev) {
    return rb_read_available(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_rx(const usart_dev_t *dev) {
    rb_reset(dev->rb);
}

/**
 * @brief Discard the contents of a serial port's RX buffer.
 * @param dev Serial port whose buffer to empty.
 */
static inline void usart_reset_tx(const usart_dev_t *dev) {
    rb_reset(dev->wb);
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif
