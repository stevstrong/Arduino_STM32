/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2012 LeafLabs, LLC.
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
 * @file libmaple/usart_private.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Private USART header.
 */

#ifndef _LIBMAPLE_USART_PRIVATE_H_
#define _LIBMAPLE_USART_PRIVATE_H_

#include <libmaple/ring_buffer.h>
#include <libmaple/usart.h>

static inline void usart_irq(const usart_dev_t * udev)
{
    /* Handling RXNEIE and TXEIE interrupts. 
     * See table 198 (sec 27.4, p809) in STM document RM0008 rev 15.
     * We enable RXNEIE. */

	register usart_reg_map * regs = udev->regs;

	// Receive part.. RXNE signifies availability of a byte in DR.
    if ((regs->CR1 & USART_CR1_RXNEIE) && (regs->SR & USART_SR_RXNE)) {
        if( (regs->SR & USART_SR_FE) || (regs->SR & USART_SR_PE) ) {
           // framing error or parity error
           regs->DR; //read and throw away the data, this clears FE and PE as well
       } else {
            /* By default, push bytes around in the ring buffer. */
            rb_write_safe(udev->rb, (uint8) regs->DR);
       }
    }
	// Transmit part. TXE signifies readiness to send a byte to DR
    if ((regs->CR1 & USART_CR1_TXEIE) && (regs->SR & USART_SR_TXE))
    {
    	register int val = rb_read_safe(udev->wb);
        if (val!=-1)
            regs->DR = (uint8_t)val;
        else
            regs->CR1 &= ~((uint32)USART_CR1_TXEIE); // disable TXEIE
    }
}

uint32 _usart_clock_freq(const usart_dev_t *dev);

#endif
