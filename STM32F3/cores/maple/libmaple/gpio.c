/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file libmaple/gpio.c
 * @author F3-port by Hanspeter Portner <dev@open-music-kontrollers.ch>
 * @brief STM32F3 GPIO.
 */

#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/bitband.h>
#include "board/board.h"

/*
 * GPIO devices
 */

/** GPIO port A device. */
const gpio_dev gpioa = {
    .regs      = GPIOA_BASE,
    .clk_id    = RCC_GPIOA,
    .exti_port = EXTI_PA,
};

/** GPIO port B device. */
const gpio_dev gpiob = {
    .regs      = GPIOB_BASE,
    .clk_id    = RCC_GPIOB,
    .exti_port = EXTI_PB,
};

/** GPIO port C device. */
const gpio_dev gpioc = {
    .regs      = GPIOC_BASE,
    .clk_id    = RCC_GPIOC,
    .exti_port = EXTI_PC,
};

const gpio_dev * const gpio_devices[] = {GPIOA, GPIOB, GPIOC};

/*
 * GPIO routines
 */

/**
 * Initialize a GPIO device.
 *
 * Enables the clock for and resets the given device.
 *
 * @param dev GPIO device to initialize.
 */
void gpio_init(const gpio_dev *dev) {
    rcc_clk_enable(dev->clk_id);
    rcc_reset_dev(dev->clk_id);
}

/**
 * Initialize and reset all available GPIO devices.
 */
void gpio_init_all(void) {
    gpio_init(GPIOA);
    gpio_init(GPIOB);
    gpio_init(GPIOC);
}

/**
 * @brief Set the mode of a GPIO pin.
 * @param dev GPIO device.
 * @param bit Bit on dev whose mode to set, 0--15.
 * @param mode Mode to set the pin to.
 * @param flags Flags to modify basic mode configuration
 */
void gpio_set_mode(uint8 pin, gpio_pin_mode mode)
{
    gpio_reg_map *regs = digitalPinToPort(pin)->regs;
	uint8 bit = (pin&0x0F);
    uint8 shift = bit<<1;

    regs->MODER      = (regs->MODER   & ~(3 << shift))  |  (((mode >> 0) & 3)  << shift);
    regs->PUPDR      = (regs->PUPDR   & ~(3 << shift))  |  (((mode >> 2) & 3)  << shift);
    regs->OSPEEDR    = (regs->OSPEEDR & ~(3 << shift))  |  (((mode >> 4) & 3)  << shift);
    /* Output type */
    bb_peri_set_bit(&regs->OTYPER, bit, (mode>>6)&1);
}

/**
 * @brief Set a pin's alternate function.
 *
 * The pin must have its mode set to GPIO_MODE_AF for this to take
 * effect.
 *
 * @param dev Device whose pin to configure.
 * @param bit Pin whose alternate function to set.
 * @param af  Alternate function to use for pin.
 * @see gpio_set_modef()
 */
void gpio_set_af(uint8 pin, gpio_af af)
{
    gpio_reg_map *regs = digitalPinToPort(pin)->regs;
    uint8 bit = pin&0x0F;

    regs->AFR[bit>>3] = (regs->AFR[bit>>3] & ~(0xF << ((bit&7)<<2))) | (af << ((bit&7)<<2));
}

/**
 * @brief Enable or disable the JTAG and SW debug ports.
 * @param config Desired debug port configuration
 * @see gpio_debug_cfg
 */
void gpio_cfg_debug_ports(gpio_debug_cfg config) {
    switch ( config ) {
        case GPIO_DEBUG_FULL_SWJ:
            gpio_set_af(PA13, GPIO_AF_SYSTEM);
            gpio_set_af(PA14, GPIO_AF_SYSTEM);
            gpio_set_af(PA15, GPIO_AF_SYSTEM);
            gpio_set_af(PB3,  GPIO_AF_SYSTEM);
            gpio_set_af(PB4,  GPIO_AF_SYSTEM);
            break;
        case GPIO_DEBUG_FULL_SWJ_NO_NJRST:
            gpio_set_af(PA13, GPIO_AF_SYSTEM);
            gpio_set_af(PA14, GPIO_AF_SYSTEM);
            gpio_set_af(PA15, GPIO_AF_SYSTEM);
            gpio_set_af(PB3,  GPIO_AF_SYSTEM);
            gpio_set_mode(PB4, GPIO_INPUT);
            break;
        case GPIO_DEBUG_SW_ONLY:
            gpio_set_af(PA13, GPIO_AF_SYSTEM);
            gpio_set_af(PA14, GPIO_AF_SYSTEM);
            gpio_set_mode(PA15, GPIO_INPUT);
            gpio_set_mode(PB3, GPIO_INPUT);
            gpio_set_mode(PB4, GPIO_INPUT);
            break;
        case GPIO_DEBUG_NONE:
            gpio_set_mode(PA13, GPIO_INPUT);
            gpio_set_mode(PA14, GPIO_INPUT);
            gpio_set_mode(PA15, GPIO_INPUT);
            gpio_set_mode(PB3, GPIO_INPUT);
            gpio_set_mode(PB4, GPIO_INPUT);
            break;
        default:
            break;
    }
}
