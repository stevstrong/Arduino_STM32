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
 * @file libmaple/include/libmaple/gpio.h
 * @brief General Purpose I/O (GPIO) interace.
 */

#ifndef _LIBMAPLE_GPIO_H_
#define _LIBMAPLE_GPIO_H_

#ifdef __cplusplus
extern "C"{
#endif

/*
 * Note: Series header must define:
 * - enum gpio_pin_mode (TODO think harder about portability here)
 */

#include "series/gpio.h"
#include <libmaple/libmaple_types.h>
#include <libmaple/rcc.h>
#include <libmaple/exti.h>

/*
 * Device type
 */

/** GPIO device type */
typedef struct gpio_dev_t {
    gpio_reg_map *regs;         /**< Register map */
    rcc_clk_id    clk_id;       /**< RCC clock information */
    /**
     * @brief (Deprecated) External interrupt port.
     * Instead of dev->exti_port, use gpio_exti_port(dev).
     */
    exti_cfg      exti_port;
} gpio_dev_t;

extern const gpio_dev_t gpioa;
#define GPIOA (&gpioa)
extern const gpio_dev_t gpiob;
#define GPIOB (&gpiob)
extern const gpio_dev_t gpioc;
#define GPIOC (&gpioc)
#if STM32_NR_GPIO_PORTS > 3
extern const gpio_dev_t gpiod;
#define GPIOD (&gpiod)
#endif
#if STM32_NR_GPIO_PORTS > 4
extern const gpio_dev_t gpioe;
#define GPIOE (&gpioe)
extern const gpio_dev_t gpiof;
#define GPIOF (&gpiof)
extern const gpio_dev_t gpiog;
#define GPIOG (&gpiog)
#endif

extern const gpio_dev_t * const gpio_devs[3];

/*
 * Portable routines
 */
static inline void enableDebugPorts() { afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); }
static inline void disableDebugPorts() { afio_cfg_debug_ports(AFIO_DEBUG_NONE); }

inline void gpio_init(const gpio_dev_t *dev) {
    rcc_clk_enable(dev->clk_id);
    rcc_reset_dev(dev->clk_id);
}

void gpio_init_all(void);
/* TODO flags argument version? */
void gpio_set_mode(const gpio_dev_t *dev, uint8 bit, gpio_pin_mode mode);
gpio_pin_mode gpio_get_mode(const gpio_dev_t *dev, uint8 bit);

static inline void gpio_set_pin_mode(uint8 pin, gpio_pin_mode mode) {
	gpio_set_mode(gpio_devs[pin/16], pin%16, mode);
}
/**
 * @brief Get a GPIO port's corresponding EXTI port configuration.
 * @param dev GPIO port whose exti_cfg to return.
 */
static inline exti_cfg gpio_exti_port(gpio_dev_t *dev) {
    return (exti_cfg)(EXTI_PA + (dev->clk_id - RCC_GPIOA));
}

/**
 * Set or reset a GPIO pin.
 *
 * Pin must have previously been configured to output mode.
 *
 * @param dev GPIO device whose pin to set.
 * @param pin Pin on to set or reset
 * @param val If true, set the pin.  If false, reset the pin.
 */
static inline void gpio_write_bit(const gpio_dev_t *dev, uint8 bit, uint8 val) {
    val = !val;          /* "set" bits are lower than "reset" bits  */
    dev->regs->BSRR = (1U << bit) << (16 * val);
}

static inline void gpio_write_pin(uint8 pin, uint8 val) {
   gpio_write_bit(gpio_devs[pin/16], pin%16, val);
}

/**
 * Determine whether or not a GPIO pin is set.
 *
 * Pin must have previously been configured to input mode.
 *
 * @param dev GPIO device whose pin to test.
 * @param pin Pin on dev to test.
 * @return True if the pin is set, false otherwise.
 */
static inline uint32 gpio_read_bit(const gpio_dev_t *dev, uint8 pin) {
    return dev->regs->IDR & (1U << pin);
}

static inline uint32 gpio_read_pin(uint8 pin) {
   return gpio_read_bit(gpio_devs[pin/16], pin%16);
}

/**
 * Toggle a pin configured as output push-pull.
 * @param dev GPIO device.
 * @param pin Pin on dev to toggle.
 */
static inline void gpio_toggle_bit(const gpio_dev_t *dev, uint8 bit) {
    dev->regs->ODR = dev->regs->ODR ^ (1U << bit);
}

static inline void gpio_toggle_pin(uint8 pin) {
	gpio_toggle_bit(gpio_devs[pin/16], (pin%16));
}

#ifdef __cplusplus
}
#endif

#endif
