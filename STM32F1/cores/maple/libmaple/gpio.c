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
 * @file libmaple/gpio.c
 * @brief STM32F1 GPIO support.
 */

#include "gpio.h"
#include "timer.h"

/*
 * GPIO devices
 */

/** GPIO port A device. */
const gpio_dev_t gpioa = {
    .regs      = GPIOA_BASE,
    .clk_id    = RCC_GPIOA,
    .exti_port = EXTI_PA,
};

/** GPIO port B device. */
const gpio_dev_t gpiob = {
    .regs      = GPIOB_BASE,
    .clk_id    = RCC_GPIOB,
    .exti_port = EXTI_PB,
};

/** GPIO port C device. */
const gpio_dev_t gpioc = {
    .regs      = GPIOC_BASE,
    .clk_id    = RCC_GPIOC,
    .exti_port = EXTI_PC,
};

#if STM32_NR_GPIO_PORTS > 3
/** GPIO port D device. */
const gpio_dev_t gpiod = {
    .regs      = GPIOD_BASE,
    .clk_id    = RCC_GPIOD,
    .exti_port = EXTI_PD,
};
#endif
#if STM32_NR_GPIO_PORTS > 4
/** GPIO port E device. */
const gpio_dev_t gpioe = {
    .regs      = GPIOE_BASE,
    .clk_id    = RCC_GPIOE,
    .exti_port = EXTI_PE,
};

/** GPIO port F device. */
const gpio_dev_t gpiof = {
    .regs      = GPIOF_BASE,
    .clk_id    = RCC_GPIOF,
    .exti_port = EXTI_PF,
};

/** GPIO port G device. */
const gpio_dev_t gpiog = {
    .regs      = GPIOG_BASE,
    .clk_id    = RCC_GPIOG,
    .exti_port = EXTI_PG,
};
#endif

const gpio_dev_t* const gpio_devs[] = {
	GPIOA,
	GPIOB,
	GPIOC,
#if STM32_NR_GPIO_PORTS > 3
	GPIOD,
#endif
#if STM32_NR_GPIO_PORTS > 4
	GPIOE,
	GPIOF,
	GPIOG,
#endif
};
/*
 * GPIO routines
 */

/**
 * Initialize and reset all available GPIO devices.
 */
void gpio_init_all(void) {
    gpio_init(GPIOA);
    gpio_init(GPIOB);
    gpio_init(GPIOC);
//    gpio_init(GPIOD);
#if STM32_NR_GPIO_PORTS > 4
    gpio_init(GPIOE);
    gpio_init(GPIOF);
    gpio_init(GPIOG);
#endif
}

/**
 * Set the mode of a GPIO pin.
 *
 * @param dev GPIO device.
 * @param pin Pin on the device whose mode to set, 0--15.
 * @param mode General purpose or alternate function mode to set the pin to.
 * @see gpio_pin_mode
 */
void gpio_set_mode(const gpio_dev_t *dev, uint8 bit, gpio_pin_mode mode) {
    gpio_reg_map *regs = dev->regs;
    __IO uint32 *cr = &regs->CRL + (bit >> 3);
    uint32 shift = (bit & 0x7) * 4;
    uint32 tmp = *cr;

    tmp &= ~(0xF << shift);
    tmp |= (mode == GPIO_INPUT_PU ? GPIO_INPUT_PD : mode) << shift;
    *cr = tmp;

    if (mode == GPIO_INPUT_PD) {
        regs->ODR &= ~(1U << bit);
    } else if (mode == GPIO_INPUT_PU) {
        regs->ODR |= (1U << bit);
    }
}
//-----------------------------------------------------------------------------
gpio_pin_mode gpio_get_mode(const gpio_dev_t *dev, uint8 pin) {
    gpio_reg_map *regs = dev->regs;
    __IO uint32 *cr = &regs->CRL + (pin >> 3);
    uint32 shift = (pin & 0x7) * 4;

	uint32 crMode = (*cr>>shift) & 0x0F;
	
	// could be pull up or pull down. Nee to check the ODR
	if (crMode==GPIO_INPUT_PD && ((regs->ODR >> pin) & 0x01) !=0 )
	{
		crMode = GPIO_INPUT_PU;
	}
	
    return(crMode);
}

/*
 * AFIO
 */

/**
 * @brief Initialize the AFIO clock, and reset the AFIO registers.
 */
void afio_init(void) {
    rcc_clk_enable(RCC_AFIO);
    rcc_reset_dev(RCC_AFIO);
}

#define AFIO_EXTI_SEL_MASK 0xF

/**
 * @brief Perform an alternate function remap.
 * @param remapping Remapping to perform.
 */
void afio_remap(afio_remap_peripheral remapping) {
    if (remapping & AFIO_REMAP_USE_MAPR2) {
        remapping &= ~AFIO_REMAP_USE_MAPR2;
        AFIO_BASE->MAPR2 |= remapping;
    } else {
        AFIO_BASE->MAPR |= remapping;
    }
}

int8_t afio_is_remapped(afio_remap_peripheral remapping) {
    if (remapping & AFIO_REMAP_USE_MAPR2) {
        remapping &= ~AFIO_REMAP_USE_MAPR2;
        return AFIO_BASE->MAPR2 & remapping;
    } else {
        return AFIO_BASE->MAPR & remapping;
    }
}
