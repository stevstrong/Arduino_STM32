/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
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
 * @file libmaple/stm32f1/include/series/spi.h
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief STM32F1 SPI/I2S series header.
 */

#ifndef _LIBMAPLE_STM32F1_SPI_H_
#define _LIBMAPLE_STM32F1_SPI_H_

#include <libmaple/libmaple_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register maps
 */

/** SPI register map type. */
typedef struct spi_reg_map {
    __IO uint32 CR1;            /**< Control register 1 */
    __IO uint32 CR2;            /**< Control register 2 */
    __IO uint32 SR;             /**< Status register */
    __IO uint32 DR;             /**< Data register */
    __IO uint32 CRCPR;          /**< CRC polynomial register */
    __IO uint32 RXCRCR;         /**< RX CRC register */
    __IO uint32 TXCRCR;         /**< TX CRC register */
    __IO uint32 I2SCFGR;        /**< I2S configuration register */
    __IO uint32 I2SPR;          /**< I2S prescaler register */
} spi_reg_map;

/*
 * Register map base pointers
 */

#define SPI1_BASE               ((spi_reg_map*)0x40013000)
#define SPI2_BASE               ((spi_reg_map*)0x40003800)
#define SPI3_BASE               ((spi_reg_map*)0x40003C00)

/*
 * Devices
 */

/** SPI device type */
typedef struct spi_dev {
    spi_reg_map *regs;          /**< Register map */
    rcc_clk_id clk_id;          /**< RCC clock information */
    nvic_irq_num irq_num;       /**< NVIC interrupt number */
} spi_dev;

extern spi_dev spi1;
extern spi_dev spi2;
#define SPI1 (&spi1)
#define SPI2 (&spi2)

#if defined(STM32_HIGH_DENSITY) || defined(STM32_XL_DENSITY)
extern spi_dev spi3;
#define SPI3 (&spi3)
#endif

typedef struct spi_pins {
    uint8 nss;
    uint8 sck;
    uint8 miso;
    uint8 mosi;
} spi_pins;

/*
 * Routines
 */

/* spi_gpio_cfg(): Backwards compatibility shim to spi_config_gpios() */
struct gpio_dev;
extern void spi_config_gpios(uint8 as_master, const spi_pins * pins);
extern void spi_release_gpios(uint8 as_master, const spi_pins * pins);

/**
 * @brief Deprecated. Use spi_config_gpios() instead.
 * @see spi_config_gpios()
 */
static inline void spi_gpio_cfg(uint8 as_master, const spi_pins * pins)
{
    /* We switched style globally to foo_config_gpios() and always
     * taking a foo_dev* argument (that last bit is the important
     * part) after this function was written.
     *
     * However, spi_config_gpios() just ignores the spi_dev* on F1, so
     * we can still keep this around for older code. */
    spi_config_gpios(as_master, pins);
}

#ifdef __cplusplus
}
#endif

#endif
