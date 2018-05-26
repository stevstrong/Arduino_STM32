/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
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
 * @file libmaple/exti.c
 * @brief External interrupt control routines
 */

#include <libmaple/exti.h>
#include <libmaple/libmaple.h>
#include <libmaple/nvic.h>
#include <libmaple/bitband.h>

static inline void dispatch_single_exti(uint32 exti_num);
static inline void dispatch_extis(uint32 start, uint32 stop);

/*
 * Internal state
 */

static voidFuncPtr exti_handlers[16] = { NULL };


/*
 * Portable routines
 */

/**
 * @brief Register a handler with an argument to run upon external interrupt.
 *
 * This function assumes that the interrupt request corresponding to
 * the given external interrupt is masked.
 *
 * @param num     External interrupt line number.
 * @param port    Port to use as source input for external interrupt.
 * @param handler Function handler to execute when interrupt is triggered.
 * @param arg     Argument to pass to the interrupt handler.
 * @param mode    Type of transition to trigger on, one of:
 *                EXTI_RISING, EXTI_FALLING, EXTI_RISING_FALLING.
 * @see exti_num
 * @see exti_cfg
 * @see voidFuncPtr
 * @see exti_trigger_mode
 */
void exti_attach_callback(exti_num num,
                          exti_cfg port,
                          voidFuncPtr handler,
                          exti_trigger_mode mode)
{
    if (handler==NULL) return;

    /* Register the handler */
    exti_handlers[num] = handler;

    /* Set trigger mode */
    switch (mode) {
    case EXTI_RISING:
        bb_peri_set_bit(&EXTI_BASE->RTSR, num, 1);
        break;
    case EXTI_FALLING:
        bb_peri_set_bit(&EXTI_BASE->FTSR, num, 1);
        break;
    case EXTI_RISING_FALLING:
        bb_peri_set_bit(&EXTI_BASE->RTSR, num, 1);
        bb_peri_set_bit(&EXTI_BASE->FTSR, num, 1);
        break;
    }

    /* Use the chip-specific exti_select() to map num to port */
    exti_select(num, port);

    /* Unmask external interrupt request */
    bb_peri_set_bit(&EXTI_BASE->IMR, num, 1);

    /* Enable the interrupt line */
    switch(num)
    {
        case EXTI0:
            nvic_irq_enable(NVIC_EXTI0);
            break;
        case EXTI1:
            nvic_irq_enable(NVIC_EXTI1);
            break;
        case EXTI2:
            nvic_irq_enable(NVIC_EXTI2);
            break;
        case EXTI3:
            nvic_irq_enable(NVIC_EXTI3);
            break;
        case EXTI4:
            nvic_irq_enable(NVIC_EXTI4);
            break;
        case EXTI5:
        case EXTI6:
        case EXTI7:
        case EXTI8:
        case EXTI9:
            nvic_irq_enable(NVIC_EXTI_9_5);
            break;
        case EXTI10:
        case EXTI11:
        case EXTI12:
        case EXTI13:
        case EXTI14:
        case EXTI15:
            nvic_irq_enable(NVIC_EXTI_15_10);
            break;
    }
}

/**
 * @brief Unregister an external interrupt handler
 * @param num External interrupt line to disable.
 * @see exti_num
 */
void exti_detach_interrupt(exti_num num) {
    /* First, mask the interrupt request */
    bb_peri_set_bit(&EXTI_BASE->IMR, num, 0);

    /* Then, clear the trigger selection registers */
    bb_peri_set_bit(&EXTI_BASE->FTSR, num, 0);
    bb_peri_set_bit(&EXTI_BASE->RTSR, num, 0);

    /* Finally, unregister the user's handler */
    exti_handlers[num] = NULL;
}

/*
 * Private routines
 */

void exti_do_select(__IO uint32 *exti_cr, exti_num num, exti_cfg port) {
    uint32 shift = 4 * (num % 4);
    uint32 cr = *exti_cr;
    cr &= ~(0xF << shift);
    cr |= port << shift;
    *exti_cr = cr;
}

/*
 * Auxiliary functions
 */

/* Clear the pending bits for EXTIs whose bits are set in exti_msk.
 *
 * If a pending bit is cleared as the last instruction in an ISR, it
 * won't actually be cleared in time and the ISR will fire again.  To
 * compensate, this function NOPs for 2 cycles after clearing the
 * pending bits to ensure it takes effect. */
static inline __always_inline void clear_pending_msk(uint32 exti_msk) {
    EXTI_BASE->PR = exti_msk;
    asm volatile("nop");
    asm volatile("nop");
}

/* This dispatch routine is for non-multiplexed EXTI lines only; i.e.,
 * it doesn't check EXTI_PR. */
static inline __always_inline void dispatch_single_exti(uint32 exti)
{
    voidFuncPtr handler = exti_handlers[exti];
    if (handler)
        handler();
    clear_pending_msk(1U << exti);
}

/* Dispatch routine for EXTIs which share an IRQ. */
static inline __always_inline void dispatch_extis(uint32 start, uint32 stop) {
    uint32 pr = EXTI_BASE->PR;
    uint32 handled_msk = 0;
    uint32 exti;

    /* Dispatch user handlers for pending EXTIs. */
    for (exti = start; exti <= stop; exti++) {
        uint32 eb = (1U << exti);
        if (pr & eb) {
            voidFuncPtr handler = exti_handlers[exti];
            if (handler) {
                handler();
                handled_msk |= eb;
            }
        }
    }

    /* Clear the pending bits for handled EXTIs. */
    clear_pending_msk(handled_msk);
}

/*
 * Interrupt handlers
 */

__weak void __irq_exti0(void) {
    dispatch_single_exti(EXTI0);
}

__weak void __irq_exti1(void) {
    dispatch_single_exti(EXTI1);
}

__weak void __irq_exti2(void) {
    dispatch_single_exti(EXTI2);
}

__weak void __irq_exti3(void) {
    dispatch_single_exti(EXTI3);
}

__weak void __irq_exti4(void) {
    dispatch_single_exti(EXTI4);
}

__weak void __irq_exti9_5(void) {
    dispatch_extis(5, 9);
}

__weak void __irq_exti15_10(void) {
    dispatch_extis(10, 15);
}
