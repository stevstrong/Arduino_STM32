/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file libmaple/usart.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>,
 *         Perry Hung <perry@leaflabs.com>
 * @brief Portable USART routines
 */

#include "usart.h"



/**
 * @brief Initialize a serial port.
 * @param dev         Serial port to be initialized
 */
void usart_init(const usart_dev_t *dev) {
    rb_reset(dev->rb);
    rb_reset(dev->wb);
    rcc_clk_enable(dev->clk_id);
    nvic_irq_enable(dev->irq_num);
}

/**
 * @brief Enable a serial port.
 *
 * USART is enabled in single buffer transmission mode, multibuffer
 * receiver mode, 8n1.
 *
 * Serial port must have a baud rate configured to work properly.
 *
 * @param dev Serial port to enable.
 * @see usart_set_baud_rate()
 */
void usart_enable(const usart_dev_t *dev) {
    usart_reg_map *regs = dev->regs;
    regs->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);// don't change the word length etc, and 'or' in the patten not overwrite |USART_CR1_M_8N1);
    regs->CR1 |= USART_CR1_UE;
}

/**
 * @brief Turn off a serial port.
 * @param dev Serial port to be disabled
 */
void usart_disable(const usart_dev_t *dev) {
    /* FIXME this misbehaves (on F1) if you try to use PWM on TX afterwards */
    usart_reg_map *regs = dev->regs;

    while(!rb_is_empty(dev->wb))
        ; // wait for TX completed
    /* TC bit must be high before disabling the USART */
    while((regs->CR1 & USART_CR1_UE) && !(regs->SR & USART_SR_TC))
        ;

    /* Disable UE */
    regs->CR1 &= ~USART_CR1_UE;

    // Clean up buffer
    rb_reset(dev->rb);
    rb_reset(dev->wb);
}

/**
 * @brief Nonblocking USART transmit
 * @param dev Serial port to transmit over
 * @param buf Buffer to transmit
 * @param len Maximum number of bytes to transmit
 * @return Number of bytes transmitted
 */
uint16 usart_tx(const usart_dev_t *dev, const uint8 *buf, uint16 len)
{
    uint16 txed = 0;
	while ( txed < len )
	{
		if ( !rb_write_safe(dev->wb, *buf++) ) // checks for buffer full
			break;
		txed++;
	}
	dev->regs->CR1 |= USART_CR1_TXEIE; // enable Tx IRQ

    return txed;
}

/**
 * @brief Nonblocking USART receive.
 * @param dev Serial port to receive bytes from
 * @param buf Buffer to store received bytes into
 * @param len Maximum number of bytes to store
 * @return Number of bytes received
 */
uint16 usart_rx(const usart_dev_t *dev, uint8 *buf, uint16 len) {
    uint16 rxed = 0;
    while (usart_rx_available(dev) && rxed < len) {
        *buf++ = usart_getc(dev);
        rxed++;
    }
    return rxed;
}

/**
 * @brief Transmit an unsigned integer to the specified serial port in
 *        decimal format.
 *
 * This function blocks until the integer's digits have been
 * completely transmitted.
 *
 * @param dev Serial port to send on
 * @param val Number to print
 */
void usart_putudec(const usart_dev_t *dev, uint32 val) {
    char digits[12];
    int i = 0;

    do {
        digits[i++] = val % 10 + '0';
        val /= 10;
    } while (val > 0);

    while (--i >= 0) {
        usart_putc(dev, digits[i]);
    }
}
