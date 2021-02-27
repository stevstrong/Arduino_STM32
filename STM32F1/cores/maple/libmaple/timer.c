
/******************************************************************************
 * The MIT License
 *
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
 * @file   libmaple/timer.c
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief  Portable timer routines.
 */
#include "libmaple_types.h"
#include <libmaple/timer.h>
#include <libmaple/stm32.h>
#include "timer_private.h"

static void disable_channel(timer_dev *dev, uint8 channel);
static void encoder_mode(timer_dev *dev); //CARLOS

static void enable_irq(timer_dev *dev, timer_interrupt_id iid);

/*
 * Devices
 *
 * Defer to the timer_private API for declaring these.
 */

/** Timer 1 device (advanced) */
timer_dev timer1 = ADVANCED_TIMER(1);
/** Timer 2 device (general-purpose) */
timer_dev timer2 = GENERAL_TIMER(2);
/** Timer 3 device (general-purpose) */
timer_dev timer3 = GENERAL_TIMER(3);
/** Timer 4 device (general-purpose) */
timer_dev timer4 = GENERAL_TIMER(4);
/** Timer 5 device (general-purpose) */
timer_dev timer5 = GENERAL_TIMER(5);
#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
/** Timer 6 device (basic) */
timer_dev timer6 = BASIC_TIMER(6);
/** Timer 7 device (basic) */
timer_dev timer7 = BASIC_TIMER(7);
/** Timer 8 device (advanced) */
timer_dev timer8 = ADVANCED_TIMER(8);
#endif
#if defined(STM32_XL_DENSITY)
/** Timer 9 device (general-purpose) */
timer_dev timer9 = RESTRICTED_GENERAL_TIMER(9, TIMER_DIER_TIE_BIT);
/** Timer 10 device (general-purpose) */
timer_dev timer10 = RESTRICTED_GENERAL_TIMER(10, TIMER_DIER_CC1IE_BIT);
/** Timer 11 device (general-purpose) */
timer_dev timer11 = RESTRICTED_GENERAL_TIMER(11, TIMER_DIER_CC1IE_BIT);
/** Timer 12 device (general-purpose) */
timer_dev timer12 = RESTRICTED_GENERAL_TIMER(12, TIMER_DIER_TIE_BIT);
/** Timer 13 device (general-purpose) */
timer_dev timer13 = RESTRICTED_GENERAL_TIMER(13, TIMER_DIER_CC1IE_BIT);
/** Timer 14 device (general-purpose) */
timer_dev timer14 = RESTRICTED_GENERAL_TIMER(14, TIMER_DIER_CC1IE_BIT);
#endif


//-----------------------------------------------------------------------------
timer_info const timer_map[32] =
{
	(uint32)TIMER2 + 1, // PA0
	(uint32)TIMER2 + 2, // PA1
	(uint32)TIMER2 + 3, // PA2
	(uint32)TIMER2 + 4, // PA3
	(uint32)NULL, // PA4
	(uint32)NULL, // PA5
	(uint32)TIMER3 + 1, // PA6
	(uint32)TIMER3 + 2, // PA7
	(uint32)TIMER1 + 1, // PA8
	(uint32)TIMER1 + 2, // PA9
	(uint32)TIMER1 + 3, // PA10
	(uint32)TIMER1 + 4, // PA11
	(uint32)NULL, // PA12
	(uint32)NULL, // PA13
	(uint32)NULL, // PA14
	(uint32)NULL, // PA15

	(uint32)TIMER3 + 3, // PB0
	(uint32)TIMER3 + 4, // PB1
	(uint32)NULL, // PB2
	(uint32)NULL, // PB3
	(uint32)NULL, // PB4
	(uint32)NULL, // PB5
	(uint32)TIMER4 + 1, // PB6
	(uint32)TIMER4 + 2, // PB7
	(uint32)TIMER4 + 3, // PB8
	(uint32)TIMER4 + 4, // PB9
	(uint32)NULL, // PB10
	(uint32)NULL, // PB11
	(uint32)NULL, // PB12
	(uint32)NULL, // PB13
	(uint32)NULL, // PB14
	(uint32)NULL, // PB15
/*
    {NULL, 0}, // PC0 
    {NULL, 0}, // PC1 
    {NULL, 0}, // PC2 
    {NULL, 0}, // PC3 
    {NULL, 0}, // PC4 
    {NULL, 0}, // PC5 
    {NULL, 0}, // PC6 
    {NULL, 0}, // PC7 
    {NULL, 0}, // PC8 
    {NULL, 0}, // PC9 
    {NULL, 0}, // PC10
    {NULL, 0}, // PC11
    {NULL, 0}, // PC12
    {NULL, 0}, // PC13	
    {NULL, 0}, // PC14
    {NULL, 0}, // PC15
*/
};



/*
 * Routines
 */

/**
 * @brief Call a function on timer devices.
 * @param fn Function to call on each timer device.
 */
void timer_foreach(void (*fn)(timer_dev*))
{
    fn(TIMER1);
    fn(TIMER2);
    fn(TIMER3);
    fn(TIMER4);
#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
    fn(TIMER5);
    fn(TIMER6);
    fn(TIMER7);
    fn(TIMER8);
#endif
#if defined(STM32_XL_DENSITY)
    fn(TIMER9);
    fn(TIMER10);
    fn(TIMER11);
    fn(TIMER12);
    fn(TIMER13);
    fn(TIMER14);
#endif
}

/**
 * Initialize a timer, and reset its register map.
 * @param dev Timer to initialize
 */
void timer_init(timer_dev *dev) {
    rcc_clk_enable(dev->clk_id);
    rcc_reset_dev(dev->clk_id);
}

/**
 * @brief Disable a timer.
 *
 * The timer will stop counting, all DMA requests and interrupts will
 * be disabled, and no state changes will be output.
 *
 * @param dev Timer to disable.
 */
void timer_disable(timer_dev *dev) {
    (dev->regs).bas->CR1 = 0;
    (dev->regs).bas->DIER = 0;
    switch (dev->type) {
    case TIMER_ADVANCED:        /* fall-through */
    case TIMER_GENERAL:
        (dev->regs).gen->CCER = 0;
        break;
    case TIMER_BASIC:
        break;
    }
}

/**
 * Sets the mode of an individual timer channel.
 *
 * Note that not all timers can be configured in every mode.  For
 * example, basic timers cannot be configured to output compare mode.
 * Be sure to use a timer which is appropriate for the mode you want.
 *
 * @param dev Timer whose channel mode to set
 * @param channel Relevant channel
 * @param mode New timer mode for channel
 */
void timer_set_mode(timer_dev *dev, uint8 channel, timer_mode mode)
{
    if ( channel==0 || channel>4 ) return;

    /* TODO decide about the basic timers */
    if (dev->type == TIMER_BASIC) return;

    switch (mode) {
    case TIMER_DISABLED:
        disable_channel(dev, channel);
        break;
    case TIMER_PWM:
        timer_disable_irq(dev, channel);
        timer_set_cc_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
        break;
    case TIMER_OUTPUT_COMPARE:
        timer_set_cc_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
        break;
    case TIMER_ENCODER: //added by CARLOS.
        encoder_mode(dev); //find a way to pass all the needed stuff on the 8bit var
        break;
    case TIMER_INPUT_CAPTURE:
		timer_set_cc_mode(dev, channel, 0, TIMER_IC_INPUT_DEFAULT);
        break;
    }
}

/**
 * @brief Determine whether a timer has a particular capture/compare channel.
 *
 * Different timers have different numbers of capture/compare channels
 * (and some have none at all). Use this function to test whether a
 * given timer/channel combination will work.
 *
 * @param dev Timer device
 * @param channel Capture/compare channel, from 1 to 4
 * @return Nonzero if dev has channel, zero otherwise.
 */
int timer_has_cc_channel(timer_dev *dev, uint8 channel) {
    /* On all currently supported series: advanced and "full-featured"
     * general purpose timers have all four channels. Of the
     * restricted general timers, timers 9 and 12 have channels 1 and
     * 2; the others have channel 1 only. Basic timers have none. */
    rcc_clk_id id = dev->clk_id;
    ASSERT((1 <= channel) && (channel <= 4));
    if (id <= RCC_TIMER5 || id == RCC_TIMER8) {
        return 1;     /* 1 and 8 are advanced, 2-5 are "full" general */
    } else if (id <= RCC_TIMER7) {
        return 0;     /* 6 and 7 are basic */
    }
    /* The rest are restricted general. */
    return (((id == RCC_TIMER9 || id == RCC_TIMER12) && channel <= 2) ||
            channel == 1);
}

/**
 * @brief Attach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to attach to; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @param handler Handler to attach to the given interrupt.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_attach_interrupt(timer_dev *dev,
                            uint8 interrupt,
                            voidFuncPtr handler) {
    dev->handlers[interrupt] = handler;
    timer_enable_irq(dev, interrupt);
    enable_irq(dev, interrupt);
}

/**
 * @brief Detach a timer interrupt.
 * @param dev Timer device
 * @param interrupt Interrupt number to detach; this may be any
 *                  timer_interrupt_id or timer_channel value appropriate
 *                  for the timer.
 * @see timer_interrupt_id
 * @see timer_channel
 */
void timer_detach_interrupt(timer_dev *dev, uint8 interrupt) {
    timer_disable_irq(dev, interrupt);
    dev->handlers[interrupt] = NULL;
}

//CARLOS 
uint8 get_direction(timer_dev *dev){
    return *bb_perip(&(dev->regs).gen->CR1, TIMER_CR1_DIR_BIT);
}



/*
 * Utilities
 */
static void disable_channel(timer_dev *dev, uint8 channel) {
    timer_detach_interrupt(dev, channel);
    timer_cc_disable(dev, channel);
}

void timer_set_cc_mode(timer_dev *dev, timer_channel channel, uint8 cc_mode, uint8 flags)
{
    timer_oc_set_mode(dev, channel, (timer_oc_mode)cc_mode, flags);
    timer_cc_enable(dev, channel);
}

//added by CARLOS.
static void encoder_mode(timer_dev *dev) {
    
    //prescaler. 
    //(dev->regs).gen->PSC = 1;

    //map inputs. 
    (dev->regs).gen->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1 | TIMER_CCMR1_CC2S_INPUT_TI2 | TIMER_CCMR1_IC2F | TIMER_CCMR1_IC1F ;

    (dev->regs).gen->SMCR = TIMER_SMCR_SMS_ENCODER3; //choose encoder 3, counting on both edges. 

    //polarity
    //(dev->regs).gen->CCER = TIMER_CCER_CC1P; //to invert the counting, only one of the inputs should be inverted.  

    //set the interval used by the encoder.
    //timer_set_reload(dev, 1000);

//    (dev->regs).gen->CR1  |=TIMER_CR1_UDIS_BIT;

    //run timer
    timer_resume(dev);
}


/* Advanced control timers have several IRQ lines corresponding to
 * different timer interrupts.
 *
 * Note: This function assumes that the only advanced timers are TIM1
 * and TIM8, and needs the obvious changes if that assumption is
 * violated by a later STM32 series. */
static void enable_adv_irq(timer_dev *dev, timer_interrupt_id id) {
    uint8 is_tim1 = dev->clk_id == RCC_TIMER1;
    nvic_irq_num irq_num;
    switch (id) {
    case TIMER_UPDATE_INTERRUPT:
        irq_num = (is_tim1 ?
                   NVIC_TIMER1_UP_TIMER10 :
                   NVIC_TIMER8_UP_TIMER13);
        break;
    case TIMER_CC1_INTERRUPT:   /* Fall through */
    case TIMER_CC2_INTERRUPT:   /* ... */
    case TIMER_CC3_INTERRUPT:   /* ... */
    case TIMER_CC4_INTERRUPT:
        irq_num = is_tim1 ? NVIC_TIMER1_CC : NVIC_TIMER8_CC;
        break;
    case TIMER_COM_INTERRUPT:   /* Fall through */
    case TIMER_TRG_INTERRUPT:
        irq_num = (is_tim1 ?
                   NVIC_TIMER1_TRG_COM_TIMER11 :
                   NVIC_TIMER8_TRG_COM_TIMER14);
        break;
    case TIMER_BREAK_INTERRUPT:
        irq_num = (is_tim1 ?
                   NVIC_TIMER1_BRK_TIMER9 :
                   NVIC_TIMER8_BRK_TIMER12);
        break;
    default:
        /* Can't happen, but placate the compiler */
        ASSERT(0);
        return;
    }
    nvic_irq_enable(irq_num);
}

/* Basic and general purpose timers have a single IRQ line, which is
 * shared by all interrupts supported by a particular timer. */
static void enable_bas_gen_irq(timer_dev *dev) {
    nvic_irq_num irq_num;
    switch (dev->clk_id) {
    case RCC_TIMER2:
        irq_num = NVIC_TIMER2;
        break;
    case RCC_TIMER3:
        irq_num = NVIC_TIMER3;
        break;
    case RCC_TIMER4:
        irq_num = NVIC_TIMER4;
        break;
    case RCC_TIMER5:
        irq_num = NVIC_TIMER5;
        break;
    case RCC_TIMER6:
        irq_num = NVIC_TIMER6;
        break;
    case RCC_TIMER7:
        irq_num = NVIC_TIMER7;
        break;
    case RCC_TIMER9:
        irq_num = NVIC_TIMER1_BRK_TIMER9;
        break;
    case RCC_TIMER10:
        irq_num = NVIC_TIMER1_UP_TIMER10;
        break;
    case RCC_TIMER11:
        irq_num = NVIC_TIMER1_TRG_COM_TIMER11;
        break;
    case RCC_TIMER12:
        irq_num = NVIC_TIMER8_BRK_TIMER12;
        break;
    case RCC_TIMER13:
        irq_num = NVIC_TIMER8_UP_TIMER13;
        break;
    case RCC_TIMER14:
        irq_num = NVIC_TIMER8_TRG_COM_TIMER14;
        break;
    default:
        ASSERT_FAULT(0);
        return;
    }
    nvic_irq_enable(irq_num);
}

static void enable_irq(timer_dev *dev, timer_interrupt_id iid)
{
    if (dev->type == TIMER_ADVANCED) {
        enable_adv_irq(dev, iid);
    } else {
        enable_bas_gen_irq(dev);
    }
}


/* Note.
 *
 * 2015/07/06 Roger Clark
 *
 * The IRQ handlers were initially in timer_f1.c however this seems to cause problems
 * in which the compiler / linker doesn't always link all the required handlers.
 * The work around was to move the handlers into this file
 */

/*
 * IRQ handlers
 *
 * Defer to the timer_private dispatch API.
 *
 * FIXME: The names of these handlers are inaccurate since XL-density
 * devices came out. Update these to match the STM32F2 names, maybe
 * using some weak symbol magic to preserve backwards compatibility if
 * possible. Once that's done, we can just move the IRQ handlers into
 * the top-level libmaple/timer.c, and there will be no need for this
 * file.
 */

__weak void __irq_tim1_brk(void) {
    dispatch_adv_brk(TIMER1);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_9_12(TIMER9);
#endif
}

__weak void __irq_tim1_up(void) {
    dispatch_adv_up(TIMER1);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_10_11_13_14(TIMER10);
#endif
}

__weak void __irq_tim1_trg_com(void) {
    dispatch_adv_trg_com(TIMER1);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_10_11_13_14(TIMER11);
#endif
}

__weak void __irq_tim1_cc(void) {
    dispatch_adv_cc(TIMER1);
}

__weak void __irq_tim2(void) {
    dispatch_general(TIMER2);
}

__weak void __irq_tim3(void) {
    dispatch_general(TIMER3);
}

__weak void __irq_tim4(void) {
    dispatch_general(TIMER4);
}

__weak void __irq_tim5(void) {
    dispatch_general(TIMER5);
}

#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
__weak void __irq_tim6(void) {
    dispatch_basic(TIMER6);
}

__weak void __irq_tim7(void) {
    dispatch_basic(TIMER7);
}

__weak void __irq_tim8_brk(void) {
    dispatch_adv_brk(TIMER8);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_9_12(TIMER12);
#endif
}

__weak void __irq_tim8_up(void) {
    dispatch_adv_up(TIMER8);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_10_11_13_14(TIMER13);
#endif
}

__weak void __irq_tim8_trg_com(void) {
    dispatch_adv_trg_com(TIMER8);
#if defined(STM32_XL_DENSITY)
    dispatch_tim_10_11_13_14(TIMER14);
#endif
}

__weak void __irq_tim8_cc(void) {
    dispatch_adv_cc(TIMER8);
}
#endif  /* defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY) */
