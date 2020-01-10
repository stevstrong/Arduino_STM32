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
 * @file rcc.c
 * @brief Implements pretty much only the basic clock setup on the
 *        stm32, clock enable/disable and peripheral reset commands.
 */

#include "libmaple.h"
#include "flash.h"
#include "gpio.h"
#include "rcc.h"
#include "bitband.h"
#include "pwr.h"

#define APB1                            RCC_APB1
#define APB2                            RCC_APB2
#define AHB1                            RCC_AHB1
#define AHB2                            RCC_AHB2
#define AHB3                            RCC_AHB3


struct rcc_dev_info {
    const rcc_clk_domain clk_domain;
    const uint8 line_num;
};

static uint32 rcc_dev_clk_speed_table[AHB3];

/* Device descriptor table, maps rcc_clk_id onto bus and enable/reset
 * register bit numbers. */
static const struct rcc_dev_info rcc_dev_table[] = {
    [RCC_GPIOA]   = { .clk_domain = AHB1, .line_num =  0 }, //*
    [RCC_GPIOB]   = { .clk_domain = AHB1, .line_num =  1 }, //*
    [RCC_GPIOC]   = { .clk_domain = AHB1, .line_num =  2 }, //*
    [RCC_GPIOD]   = { .clk_domain = AHB1, .line_num =  3 }, //*
    [RCC_GPIOE]   = { .clk_domain = AHB1, .line_num =  4 }, //*
    [RCC_GPIOF]   = { .clk_domain = AHB1, .line_num =  5 }, //*
    [RCC_GPIOG]   = { .clk_domain = AHB1, .line_num =  6 }, //*
    [RCC_GPIOH]   = { .clk_domain = AHB1, .line_num =  7 }, //*
    [RCC_GPIOI]   = { .clk_domain = AHB1, .line_num =  8 }, //*

    [RCC_CRC]     = { .clk_domain = AHB1, .line_num = 12},  //*
//  [RCC_FLITF]   = { .clk_domain = AHB1, .line_num = 15},
//  [RCC_SRAM1]   = { .clk_domain = AHB1, .line_num = 16},
//  [RCC_SRAM2]   = { .clk_domain = AHB1, .line_num = 17},
//  [RCC_BKPSRAM] = { .clk_domain = AHB1, .line_num = 18},  //*
#ifdef __CCMRAM__
    [RCC_CCMRAM]  = { .clk_domain = AHB1, .line_num = 20 }, //?
#endif
    [RCC_DMA1]    = { .clk_domain = AHB1, .line_num = 21 }, //*
    [RCC_DMA2]    = { .clk_domain = AHB1, .line_num = 22 }, //*
    [RCC_ETHMAC]  = { .clk_domain = AHB1, .line_num = 25 },
    [RCC_ETHMACTX]= { .clk_domain = AHB1, .line_num = 26 },
    [RCC_ETHMACRX]= { .clk_domain = AHB1, .line_num = 27 },
    [RCC_ETHMACPTP]={ .clk_domain = AHB1, .line_num = 28 },

    [RCC_DCMI]    = { .clk_domain = AHB2, .line_num =  0 }, //*
    [RCC_USBFS]   = { .clk_domain = AHB2, .line_num =  7 }, //*

    [RCC_FSMC]    = { .clk_domain = AHB3, .line_num =  0 }, //*

    [RCC_TIMER1]  = { .clk_domain = APB2, .line_num =  0 }, //*
    [RCC_TIMER2]  = { .clk_domain = APB1, .line_num =  0 }, //unchanged
    [RCC_TIMER3]  = { .clk_domain = APB1, .line_num =  1 }, //unchanged
    [RCC_TIMER4]  = { .clk_domain = APB1, .line_num =  2 }, //unchanged
    [RCC_TIMER5]  = { .clk_domain = APB1, .line_num =  3 }, //unchanged
    [RCC_TIMER6]  = { .clk_domain = APB1, .line_num =  4 }, //unchanged
    [RCC_TIMER7]  = { .clk_domain = APB1, .line_num =  5 }, //unchanged
    [RCC_TIMER8]  = { .clk_domain = APB2, .line_num =  1 }, //*
    [RCC_TIMER9]  = { .clk_domain = APB2, .line_num = 16 }, //*
    [RCC_TIMER10] = { .clk_domain = APB2, .line_num = 17 }, //*
    [RCC_TIMER11] = { .clk_domain = APB2, .line_num = 18 }, //*
    [RCC_TIMER12] = { .clk_domain = APB1, .line_num =  6 }, //unchanged
    [RCC_TIMER13] = { .clk_domain = APB1, .line_num =  7 }, //unchanged
    [RCC_TIMER14] = { .clk_domain = APB1, .line_num =  8 }, //unchanged
    [RCC_WDG]     = { .clk_domain = APB1, .line_num = 11},  //?
    [RCC_SPI1]    = { .clk_domain = APB2, .line_num = 12 }, //unchanged
    [RCC_SPI2]    = { .clk_domain = APB1, .line_num = 14 }, //unchanged
    [RCC_SPI3]    = { .clk_domain = APB1, .line_num = 15 }, //unchanged
    [RCC_SPI4]    = { .clk_domain = APB2, .line_num = 13 },
    [RCC_SPI5]    = { .clk_domain = APB2, .line_num = 20 },

    [RCC_USART1]  = { .clk_domain = APB2, .line_num =  4 }, //*
    [RCC_USART2]  = { .clk_domain = APB1, .line_num = 17 }, //unchanged
    [RCC_USART3]  = { .clk_domain = APB1, .line_num = 18 }, //unchanged
    [RCC_UART4]   = { .clk_domain = APB1, .line_num = 19 }, //unchanged
    [RCC_UART5]   = { .clk_domain = APB1, .line_num = 20 }, //unchanged
    [RCC_USART6]  = { .clk_domain = APB2, .line_num =  5 }, //*
    [RCC_ADC1]    = { .clk_domain = APB2, .line_num =  8 }, //*
    [RCC_ADC2]    = { .clk_domain = APB2, .line_num =  9 }, //*
    [RCC_ADC3]    = { .clk_domain = APB2, .line_num = 10 }, //*
    [RCC_SDIO]    = { .clk_domain = APB2, .line_num = 11 }, //*
    [RCC_SYSCFG]  = { .clk_domain = APB2, .line_num = 14 }, //*

    [RCC_I2C1]    = { .clk_domain = APB1, .line_num = 21 }, //unchanged
    [RCC_I2C2]    = { .clk_domain = APB1, .line_num = 22 }, //unchanged
    [RCC_I2C3]    = { .clk_domain = APB1, .line_num = 23 }, //?
    [RCC_CAN1]    = { .clk_domain = APB1, .line_num = 25 }, //?
    [RCC_CAN2]    = { .clk_domain = APB1, .line_num = 26 }, //?
    [RCC_PWR]     = { .clk_domain = APB1, .line_num = 28 }, //unchanged
    [RCC_DAC]     = { .clk_domain = APB1, .line_num = 29 }, //unchanged
};

/**
 * @brief Initialize the clock control system. Initializes the system
 *        clock source to use the PLL driven by an external oscillator
 * @param sysclk_src system clock source, must be PLL
 * @param pll_src pll clock source, must be HSE
 * @param pll_mul pll multiplier
 */

/*******************  Bits definition for FLASH_ACR register  *****************/
//#define FLASH_ACR_LATENCY                    ((uint32_t)0x00000007)
#define FLASH_ACR_LATENCY_0WS                ((uint32)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((uint32)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((uint32)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((uint32)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((uint32)0x00000004)
#define FLASH_ACR_LATENCY_5WS                ((uint32)0x00000005)
#define FLASH_ACR_LATENCY_6WS                ((uint32)0x00000006)
#define FLASH_ACR_LATENCY_7WS                ((uint32)0x00000007)

#define FLASH_ACR_PRFTEN                     ((uint32)0x00000100)
#define FLASH_ACR_ICEN                       ((uint32)0x00000200)
#define FLASH_ACR_DCEN                       ((uint32)0x00000400)
#define FLASH_ACR_ICRST                      ((uint32)0x00000800)
#define FLASH_ACR_DCRST                      ((uint32)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS              ((uint32)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS              ((uint32)0x40023C03)

typedef struct
{
  __IO uint32 ACR;      /*!< FLASH access control register, Address offset: 0x00 */
  __IO uint32 KEYR;     /*!< FLASH key register,            Address offset: 0x04 */
  __IO uint32 OPTKEYR;  /*!< FLASH option key register,     Address offset: 0x08 */
  __IO uint32 SR;       /*!< FLASH status register,         Address offset: 0x0C */
  __IO uint32 CR;       /*!< FLASH control register,        Address offset: 0x10 */
  __IO uint32 OPTCR;    /*!< FLASH option control register, Address offset: 0x14 */
} FLASH_TypeDef;

#define FLASH_R_BASE          (0x40023C00)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define RESET 0

//-----------------------------------------------------------------------------
void InitMCO1()
{
    // Turn MCO1 Master Clock Output mode
    RCC->CFGR &= RCC_CFGR_MCO1_RESET_MASK;
    RCC->CFGR |= RCC_CFGR_MCO1Source_HSE | RCC_CFGR_MCO1Div_1;
    // PA8 Output the Master Clock MCO1
    gpio_set_af_mode(PA8, GPIO_AFMODE_SYSTEM);
    gpio_set_mode(PA8, GPIO_MODE_AF | GPIO_OTYPE_PP | GPIO_OSPEED_100MHZ);
}

uint32_t SystemCoreClock;
//-----------------------------------------------------------------------------
void SetupClock72MHz()
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	/************************* PLL Parameters *************************************/
	/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
	int PLL_M = 4;
	int PLL_N = 216;

	/* SYSCLK = PLL_VCO / PLL_P */
	int PLL_P = 6;

	/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
	int PLL_Q = 9;


	/* Enable HSE */
	RCC->CR |= (uint32_t)(RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Select regulator voltage output Scale 2 mode, System frequency up to 144 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//*bb_perip(&PWR->CR, PWR_CR_VOS_BIT) = 0;

	/* HCLK = SYSCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);

	/* PCLK2 = HCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);

	/* PCLK1 = HCLK / 2*/
	rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_2);

	// save bus clock values
	rcc_dev_clk_speed_table[RCC_AHB1] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB2] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB1] = (SystemCoreClock/2);

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_2WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= ~(RCC_CFGR_SW_MASK);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK ) != RCC_CFGR_SWS_PLL);
}

//-----------------------------------------------------------------------------
void SetupClock84MHz()
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	/************************* PLL Parameters *************************************/
	// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N = 25[MHz]/25 * 336 = 336
	int PLL_M = 25;
	int PLL_N = 336;

	// SYSCLK = PLL_VCO / PLL_P = 336 / 4 = 84
	int PLL_P = 4;

	// USB OTG FS, SDIO and RNG Clock = PLL_VCO / PLLQ = 336 / 7 = 48
	int PLL_Q = 7;


	/* Enable HSE */
	RCC->CR |= (uint32_t)(RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Select regulator voltage output Scale 2 mode, System frequency up to 144 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//*bb_perip(&PWR->CR, PWR_CR_VOS_BIT) = 0;

	/* HCLK = SYSCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);

	/* PCLK2 = HCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);

	/* PCLK1 = HCLK / 2*/
	rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_2);

	// save bus clock values
	rcc_dev_clk_speed_table[RCC_AHB1] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB2] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB1] = (SystemCoreClock/2);

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= ~(RCC_CFGR_SW_MASK);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK ) != RCC_CFGR_SWS_PLL);
}

//-----------------------------------------------------------------------------
void SetupClock96MHz()
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	/************************* PLL Parameters *************************************/
	// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N = 8[MHz]/4 * 192 = 384
	int PLL_M = 4;
	int PLL_N = 192;

	// SYSCLK = PLL_VCO / PLL_P = 384 / 4 = 96
	int PLL_P = 4;

	// USB OTG FS, SDIO and RNG Clock = PLL_VCO / PLLQ = 384 / 8 = 48
	int PLL_Q = 8;


	/* Enable HSE */
	RCC->CR |= (uint32_t)(RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Select regulator voltage output Scale 2 mode, System frequency up to 144 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//*bb_perip(&PWR->CR, PWR_CR_VOS_BIT) = 0;

	/* HCLK = SYSCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);

	/* PCLK2 = HCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_1);

	/* PCLK1 = HCLK / 2*/
	rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_2);

	// save bus clock values
	rcc_dev_clk_speed_table[RCC_AHB1] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB2] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB1] = (SystemCoreClock/2);

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= ~(RCC_CFGR_SW_MASK);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK ) != RCC_CFGR_SWS_PLL);
}

//-----------------------------------------------------------------------------
void SetupClock120MHz()
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	/************************* PLL Parameters *************************************/
	/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
	int PLL_M = 8;
	int PLL_N = 240;

	/* SYSCLK = PLL_VCO / PLL_P */
	int PLL_P = 2;

	/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
	int PLL_Q = 5;


	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Select regulator voltage output Scale 2 mode, System frequency up to 144 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//*bb_perip(&PWR->CR, PWR_CR_VOS_BIT) = 0;

	/* HCLK = SYSCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);

	/* PCLK2 = HCLK / 2*/
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_2);

	/* PCLK1 = HCLK / 4*/
	rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_4);

	// save bus clock values
	rcc_dev_clk_speed_table[RCC_AHB1] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB2] = (SystemCoreClock/2);
	rcc_dev_clk_speed_table[RCC_APB1] = (SystemCoreClock/4);

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= (uint32_t)~(RCC_CFGR_SW_MASK);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK ) != RCC_CFGR_SWS_PLL);
}

//-----------------------------------------------------------------------------
void SetupClock168MHz()
{
	/******************************************************************************/
	/*            PLL (clocked by HSE) used as System clock source                */
	/******************************************************************************/
	/************************* PLL Parameters *************************************/
	/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#if CRYSTAL_FREQ==25
	int PLL_M = 25;
#elif CRYSTAL_FREQ==8
	int PLL_M = 8;
#else
	#error Crystal frequency not specified!
#endif
	int PLL_N = 336;

	/* SYSCLK = PLL_VCO / PLL_P */
	int PLL_P = 2;

	/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
	int PLL_Q = 7;


#ifdef BOARD_STM32F4_NETDUINO2PLUS
        InitMCO1();
#endif
        
	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	while (!(RCC->CR & RCC_CR_HSERDY));

	/* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	//*bb_perip(&PWR->CR, PWR_CR_VOS_BIT) = 0;

	/* HCLK = SYSCLK / 1*/
	rcc_set_prescaler(RCC_PRESCALER_AHB, RCC_AHB_SYSCLK_DIV_1);

	/* PCLK2 = HCLK / 2*/
	rcc_set_prescaler(RCC_PRESCALER_APB2, RCC_APB2_HCLK_DIV_2);

	/* PCLK1 = HCLK / 4*/
	rcc_set_prescaler(RCC_PRESCALER_APB1, RCC_APB1_HCLK_DIV_4);

	// save bus clock values
	rcc_dev_clk_speed_table[RCC_AHB1] = (SystemCoreClock/1);
	rcc_dev_clk_speed_table[RCC_APB2] = (SystemCoreClock/2);
	rcc_dev_clk_speed_table[RCC_APB1] = (SystemCoreClock/4);

	/* Configure the main PLL */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= (uint32_t)~(RCC_CFGR_SW_MASK);
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK ) != RCC_CFGR_SWS_PLL);
}

//-----------------------------------------------------------------------------
void rcc_clk_init(void)
{
	SystemCoreClock = CYCLES_PER_MICROSECOND * 1000000;

#if CYCLES_PER_MICROSECOND == 168
	  SetupClock168MHz();
#elif CYCLES_PER_MICROSECOND == 120
	  SetupClock120MHz();
#elif CYCLES_PER_MICROSECOND == 96
	  SetupClock96MHz();
#elif CYCLES_PER_MICROSECOND == 84
	  SetupClock84MHz();
#elif CYCLES_PER_MICROSECOND == 72
	  SetupClock72MHz();
#else
	#error Wrong CYCLES_PER_MICROSECOND!
#endif
}


static const __IO uint32* enable_regs[] = {
	[APB1] = &RCC->APB1ENR,
	[APB2] = &RCC->APB2ENR,
	[AHB1] = &RCC->AHB1ENR,
	[AHB2] = &RCC->AHB2ENR,
	[AHB3] = &RCC->AHB3ENR,
};
/**
 * @brief Turn on the clock line on a peripheral
 * @param id Clock ID of the peripheral to turn on.
 */
void rcc_clk_enable(rcc_clk_id id)
{
    rcc_clk_domain clk_domain = rcc_dev_clk(id);
    __IO uint32* enr = (__IO uint32*)enable_regs[clk_domain];
    uint8 lnum = rcc_dev_table[id].line_num;

    bb_peri_set_bit(enr, lnum, 1);
}

/**
 * @brief Turn on the clock line on a peripheral
 * @param id Clock ID of the peripheral to turn on.
 */
void rcc_clk_disable(rcc_clk_id id)
{
    rcc_clk_domain clk_domain = rcc_dev_clk(id);
    __IO uint32* enr = (__IO uint32*)enable_regs[clk_domain];
    uint8 lnum = rcc_dev_table[id].line_num;

    bb_peri_set_bit(enr, lnum, 0);
}

static const __IO uint32* reset_regs[] = {
	[APB1] = &RCC->APB1RSTR,
	[APB2] = &RCC->APB2RSTR,
	[AHB1] = &RCC->AHB1RSTR,
	[AHB2] = &RCC->AHB2RSTR,
	[AHB3] = &RCC->AHB3RSTR,
};
/**
 * @brief Reset a peripheral.
 * @param id Clock ID of the peripheral to reset.
 */
void rcc_reset_dev(rcc_clk_id id)
{
    rcc_clk_domain clk_domain = rcc_dev_clk(id);
    __IO void* addr = (__IO void*)reset_regs[clk_domain];
    uint8 lnum = rcc_dev_table[id].line_num;

    bb_peri_set_bit(addr, lnum, 1);
    bb_peri_set_bit(addr, lnum, 0);
}

/**
 * @brief Get a peripheral's clock domain
 * @param id Clock ID of the peripheral whose clock domain to return
 * @return Clock source for the given clock ID
 */
rcc_clk_domain rcc_dev_clk(rcc_clk_id id) {
    return rcc_dev_table[id].clk_domain;
}

/**
 * @brief Get a peripheral's clock domain speed
 * @param id Clock ID of the peripheral whose clock domain speed to return
 * @return Clock speed for the given clock ID
 */
uint32 rcc_dev_clk_speed(rcc_clk_id id) {
    return rcc_dev_clk_speed_table[rcc_dev_clk(id)];
}

/**
 * @brief Get a peripheral's timer clock domain speed
 * @param id Clock ID of the peripheral whose clock domain speed to return
 * @return Clock speed for the given clock ID
 */
uint32 rcc_dev_timer_clk_speed(rcc_clk_id id) {
    return 2*rcc_dev_clk_speed(id);
}

/**
 * @brief Set the divider on a peripheral prescaler
 * @param prescaler prescaler to set
 * @param divider prescaler divider
 */
void rcc_set_prescaler(rcc_prescaler prescaler, uint32 divider)
{
    uint32 cfgr = RCC->CFGR & ~(prescaler);
    RCC->CFGR = cfgr | divider;
}
