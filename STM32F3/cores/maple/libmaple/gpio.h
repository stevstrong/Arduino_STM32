/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
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
 * @file libmaple/stm32f3/include/series/gpio.h
 * @author F3-port by Hanspeter Portner <dev@open-music-kontrollers.ch>
 * @brief STM32F3 GPIO support.
 */

#ifndef _LIBMAPLE_STM32F3_GPIO_H_
#define _LIBMAPLE_STM32F3_GPIO_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <libmaple/libmaple_types.h>
#include <libmaple/rcc.h>
#include <libmaple/exti.h>
#include <libmaple/util.h>

/*
 * GPIO register maps and devices
 */

/** GPIO register map type */
typedef struct gpio_reg_map {
    __IO uint32 MODER;          /**< Mode register */
    __IO uint32 OTYPER;         /**< Output type register */
    __IO uint32 OSPEEDR;        /**< Output speed register */
    __IO uint32 PUPDR;          /**< Pull-up/pull-down register */
    __IO uint32 IDR;            /**< Input data register */
    __IO uint32 ODR;            /**< Output data register */
    __IO uint32 BSRR;           /**< Bit set/reset register */
    __IO uint32 LCKR;           /**< Configuration lock register */
    __IO uint32 AFR[2];         /**< Alternate function low register */
    __IO uint32 BRR;   			/**< Port bit reset register */
} gpio_reg_map;

/** GPIO port A register map base pointer */
#define GPIOA_BASE                      ((struct gpio_reg_map*)0x48000000)
/** GPIO port B register map base pointer */
#define GPIOB_BASE                      ((struct gpio_reg_map*)0x48000400)
/** GPIO port C register map base pointer */
#define GPIOC_BASE                      ((struct gpio_reg_map*)0x48000800)
/** GPIO port D register map base pointer */
#define GPIOD_BASE                      ((struct gpio_reg_map*)0x48000C00)
/** GPIO port E register map base pointer */
#define GPIOE_BASE                      ((struct gpio_reg_map*)0x48001000)
/** GPIO port F register map base pointer */
#define GPIOF_BASE                      ((struct gpio_reg_map*)0x48001400)


/*
 * Device type
 */

/** GPIO device type */
typedef struct gpio_dev {
    gpio_reg_map *regs;         /**< Register map */
    rcc_clk_id    clk_id;       /**< RCC clock information */
    /**
     * @brief (Deprecated) External interrupt port.
     * Instead of dev->exti_port, use gpio_exti_port(dev).
     */
    exti_cfg      exti_port;
} gpio_dev;


extern const gpio_dev gpioa;
extern const gpio_dev gpiob;
extern const gpio_dev gpioc;
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define GPIOC (&gpioc)

/*
 * Register bit definitions
 *
 * Currently, we only provide masks to be used for shifting for some
 * registers, rather than repeating the same values 16 times.
 */

/* Mode register */
#define GPIO_MODE_INPUT                0x0
#define GPIO_MODE_OUTPUT               0x1
#define GPIO_MODE_AF                   0x2
#define GPIO_MODE_ANALOG               0x3

/* Pull-up/pull-down register */
#define GPIO_PUPD_NONE                 (0x0<<2)
#define GPIO_PUPD_PU                   (0x1<<2)
#define GPIO_PUPD_PD                   (0x2<<2)

/* Output speed register */
#define GPIO_OSPEED_LOW                (0x0<<4) //  2MHz
#define GPIO_OSPEED_MED                (0x1<<4) // 10MHz
#define GPIO_OSPEED_HIGH               (0x3<<4) // 50MHz

/* Output type register */
#define GPIO_OTYPE_PP                  (0x0<<6)
#define GPIO_OTYPE_OD                  (0x1<<6)


/* Alternate function register low */

#define GPIO_AFRL_AF0                   (0xFU << 0)
#define GPIO_AFRL_AF1                   (0xFU << 4)
#define GPIO_AFRL_AF2                   (0xFU << 8)
#define GPIO_AFRL_AF3                   (0xFU << 12)
#define GPIO_AFRL_AF4                   (0xFU << 16)
#define GPIO_AFRL_AF5                   (0xFU << 20)
#define GPIO_AFRL_AF6                   (0xFU << 24)
#define GPIO_AFRL_AF7                   (0xFU << 28)

/* Alternate function register high */

#define GPIO_AFRH_AF8                   (0xFU << 0)
#define GPIO_AFRH_AF9                   (0xFU << 4)
#define GPIO_AFRH_AF10                  (0xFU << 8)
#define GPIO_AFRH_AF11                  (0xFU << 12)
#define GPIO_AFRH_AF12                  (0xFU << 16)
#define GPIO_AFRH_AF13                  (0xFU << 20)
#define GPIO_AFRH_AF14                  (0xFU << 24)
#define GPIO_AFRH_AF15                  (0xFU << 28)

/*
 * GPIO routines
 */

/**
 * @brief GPIO pin modes
 */
typedef enum gpio_pin_mode {
    GPIO_INPUT          = (GPIO_MODE_INPUT | GPIO_PUPD_NONE),
    GPIO_INPUT_PU       = (GPIO_MODE_INPUT | GPIO_PUPD_PU),
    GPIO_INPUT_PD       = (GPIO_MODE_INPUT | GPIO_PUPD_PD),
    GPIO_INPUT_ANALOG   = (GPIO_MODE_ANALOG | GPIO_PUPD_NONE),
    GPIO_OUTPUT         = (GPIO_MODE_OUTPUT | GPIO_OSPEED_HIGH),
    GPIO_OUTPUT_OD      = (GPIO_MODE_OUTPUT | GPIO_OTYPE_OD | GPIO_OSPEED_HIGH),
    GPIO_OUTPUT_OD_PU   = (GPIO_MODE_OUTPUT | GPIO_OTYPE_OD | GPIO_PUPD_PU | GPIO_OSPEED_HIGH),
    GPIO_AF_INPUT       = (GPIO_MODE_AF | GPIO_PUPD_NONE),
    GPIO_AF_INPUT_PU    = (GPIO_MODE_AF | GPIO_PUPD_PU),
    GPIO_AF_INPUT_PD    = (GPIO_MODE_AF | GPIO_PUPD_PD),
    GPIO_AF_OUTPUT      = (GPIO_MODE_AF | GPIO_OSPEED_HIGH),
    GPIO_AF_OUTPUT_PU   = (GPIO_MODE_AF | GPIO_PUPD_PU | GPIO_OSPEED_HIGH),
    GPIO_AF_OUTPUT_OD   = (GPIO_MODE_AF | GPIO_OTYPE_OD | GPIO_OSPEED_HIGH),
    GPIO_BIGNUMBER      = 0xfff
} gpio_pin_mode;


extern const gpio_dev * const gpio_devices[];
//static inline const gpio_dev * digitalPinToPort(uint8_t pin) { return gpio_devices[pin/16]; }
#define digitalPinToPort(P)        ( gpio_devices[P/16] )
#define digitalPinToBit(P)         ( (P&0x0F) )
#define digitalPinToBitMask(P)     ( BIT(P&0x0F) )
#define portOutputRegister(port)   ( &(port->regs->ODR) )
#define portInputRegister(port)    ( &(port->regs->IDR) )

#define portSetRegister(pin)		( &(digitalPinToPort(pin)->regs->BSRR) )
#define portClearRegister(pin)		( &(digitalPinToPort(pin)->regs->BSRR) )

#define portConfigRegister(pin)		( &(digitalPinToPort(pin)->regs->CRL) )


/**
 * @brief Set the mode of a GPIO pin.
 *
 * Calling this function is equivalent to calling gpio_set_modef(dev,
 * pin, mode, GPIO_MODE_SPEED_HIGH). Note that this overrides the
 * default speed.
 *
 * @param pin Pinon the device whose mode to set
 * @param mode Mode to set the pin to.
 */
void gpio_set_mode(uint8 pin, gpio_pin_mode mode);


/**
 * @brief Available peripheral remaps.
 * @see afio_remap()
 */
typedef enum {
    GPIO_AF_SYSTEM             = 0,
    GPIO_AF_TIM2_15_16_17      = 1,
    GPIO_AF_TIM3_4_8_15        = 2,
    GPIO_AF_TSC                = 3,
    GPIO_AF_COMP7              = 3,
    GPIO_AF_TIM1_8             = 4,
    GPIO_AF_I2C1_2             = 4,
    GPIO_AF_SPI1_2             = 5,
    GPIO_AF_I2S2_3             = 5,
    GPIO_AF_TIM8_CH1_2         = 5,
    GPIO_AF_SPI3               = 6,
    GPIO_AF_I2S3_TIM1_8        = 6,
    GPIO_AF_USART1_2_3         = 7,
    GPIO_AF_COMP1_2_3_4_5_6    = 8,
    GPIO_AF_CAN1_2             = 9,
    GPIO_AF_TIM8_15            = 9,
    GPIO_AF_TIM2_3_4_8_17      = 10,
    GPIO_AF_TIM1_8_BKIN        = 11,
    GPIO_AF_TIM1_BKIN          = 12,
    GPIO_AF_USB                = 14,
} gpio_af;

/**
 * @brief Debug port configuration
 *
 * Used to configure the behavior of JTAG and Serial Wire (SW) debug
 * ports and their associated GPIO pins.
 *
 * @see afio_cfg_debug_ports()
 */
typedef enum {
    GPIO_DEBUG_FULL_SWJ          = 0, // Full Serial Wire and JTAG debug
    GPIO_DEBUG_FULL_SWJ_NO_NJRST = 1, // Full Serial Wire and JTAG, but no NJTRST.
    GPIO_DEBUG_SW_ONLY           = 2, // Serial Wire debug only (JTAG-DP disabled, SW-DP enabled)
    GPIO_DEBUG_NONE              = 3, // No debug; all JTAG and SW pins are free for use as GPIOs.
} gpio_debug_cfg;


void gpio_set_af(uint8 pin, gpio_af af);

/*
 * Portable routines
 */

void gpio_init(const gpio_dev *dev);
void gpio_init_all(void);

/**
 * @brief Get a GPIO port's corresponding EXTI port configuration.
 * @param dev GPIO port whose exti_cfg to return.
 */
static inline exti_cfg gpio_exti_port(const gpio_dev *dev) {
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
static inline void gpio_write_bit(const gpio_dev *dev, uint8 pin, uint8 val) {
    val = !val;          /* "set" bits are lower than "reset" bits  */
    dev->regs->BSRR = (1U << pin) << (16 * val);
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
static inline uint32 gpio_read_bit(const gpio_dev *dev, uint8 pin) {
    return dev->regs->IDR & (1U << pin);
}

/**
 * Toggle a pin configured as output push-pull.
 * @param dev GPIO device.
 * @param pin Pin on dev to toggle.
 */
static inline void gpio_toggle_bit(const gpio_dev *dev, uint8 pin) {
    dev->regs->ODR = dev->regs->ODR ^ (1U << pin);
}

static inline void gpio_set_pin(uint8 pin) {
	(digitalPinToPort(pin))->regs->BSRR = (uint32)BIT(pin&0x0F);
}

static inline void gpio_clear_pin(uint8 pin) {
	(digitalPinToPort(pin))->regs->BSRR = (uint32)BIT(pin&0x0F)<<16;
}

static inline void gpio_write_pin(uint8 pin, uint8 val) {
    if (val) {
        gpio_set_pin(pin);
    } else {
        gpio_clear_pin(pin);
    }
}



#ifdef __cplusplus
}
#endif

#endif
