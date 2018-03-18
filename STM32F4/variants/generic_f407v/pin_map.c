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
 * @file   generic_f407v.cpp
 * @author ala42
 * @brief  generic_f407v board file.
 */

#ifdef __cplusplus
extern "C"{
#endif

//#include "generic_f407v.h"

//#include "fsmc.h"
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/timer.h>

#include <wirish_types.h>

/*
typedef struct stm32_pin_info {
    gpio_dev *gpio_device;      // Maple pin's GPIO device
    timer_dev *timer_device;    // Pin's timer device, if any.
    uint8 timer_channel;        // Timer channel, or 0 if none.
    uint8 adc_dev_indx;         // ADC device index, if any.
    uint8 adc_channel;          // Pin ADC channel, or ADCx if none.
} stm32_pin_info;
*/

const stm32_pin_info PIN_MAP[] =
{ // LQFP100 package pin
{GPIOA, TIMER5, 1, ADC_1,    0}, // PA0  | 23 | USART2_CTS | UART4_TX       | ETH_MII_CRS      | TIM2_CH1_ETR   | TIM5_CH1       | TIM8_ETR                      | ADC123_IN0/WKUP
{GPIOA, TIMER5, 2, ADC_1,    1}, // PA1  | 24 | USART2_RTS | UART4_RX       | ETH_RMII_REF_CLK | ETH_MII_RX_CLK | TIM5_CH2       | TIM2_CH2                      | ADC123_IN1
{GPIOA, TIMER5, 3, ADC_1,    2}, // PA2  | 25 | USART2_TX  | TIM5_CH3       | TIM9_CH1         | TIM2_CH3       | ETH_MDIO                                       | ADC123_IN2
{GPIOA, TIMER5, 4, ADC_1,    3}, // PA3  | 26 | USART2_RX  | TIM5_CH4       | TIM9_CH2         | TIM2_CH4       | OTG_HS_ULPI_D0 | ETH_MII_COL                   | ADC123_IN3
{GPIOA,   NULL, 0, ADC_1,    4}, // PA4  | 29 | SPI1_NSS   | SPI3_NSS       | USART2_CK        | DCMI_HSYNC     | OTG_HS_SOF     | I2S3_WS                       | ADC12_IN4 / DAC_OUT1
{GPIOA,   NULL, 0, ADC_1,    5}, // PA5  | 30 | SPI1_SCK   | OTG_HS_ULPI_CK | TIM2_CH1_ETR     | TIM8_CH1N                                                       | ADC12_IN5 / DAC_OUT2
{GPIOA,   NULL, 0, ADC_1,    6}, // PA6  | 31 | SPI1_MISO  | TIM8_BKIN      | TIM13_CH1        | DCMI_PIXCLK    | TIM3_CH1       | TIM1_BKIN                     | ADC12_IN6
{GPIOA,   NULL, 0, ADC_1,    7}, // PA7  | 32 | SPI1_MOSI  | TIM8_CH1N      | TIM14_CH1        | TIM3_CH2       | ETH_MII_RX_DV  | TIM1_CH1N   / ETH_RMII_CRS_DV | ADC12_IN7
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA8  | 67 | MCO1       | USART1_CK      | TIM1_CH1         | I2C3_SCL       | OTG_FS_SOF
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA9  | 68 | USART1_TX  | TIM1_CH2       | I2C3_SMBA        | DCMI_D0
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA10 | 69 | USART1_RX  | TIM1_CH3       | OTG_FS_ID        | DCMI_D1
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA11 | 70 | USART1_CTS | CAN1_RX        | TIM1_CH4         | OTG_FS_DM
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA12 | 71 | USART1_RTS | CAN1_TX        | TIM1_ETR         | OTG_FS_DP
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA13 | 72 | JTMS-SWDIO
{GPIOA,   NULL, 0,  NULL, ADCx}, // PA14 | 76 | JTCK-SWCLK
{GPIOA, TIMER2, 1,  NULL, ADCx}, // PA15 | 77 | JTDI       | SPI3_NSS       | I2S3_WS          | TIM2_CH1_ETR   | SPI1_NSS

{GPIOB, TIMER3, 3, ADC_1,    8}, // PB0  | 35 | TIM3_CH3   | TIM8_CH2N | OTG_HS_ULPI_D1 | ETH_MII_RXD2   | TIM1_CH2N      | ADC12_IN8
{GPIOB, TIMER3, 4, ADC_1,    9}, // PB1  | 36 | TIM3_CH4   | TIM8_CH3N | OTG_HS_ULPI_D2 | ETH_MII_RXD3   | TIM1_CH3N      | ADC12_IN9
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB2  | 37 | BOOT1    
{GPIOB, TIMER2, 2,  NULL, ADCx}, // PB3  | 89 | JTDO       | TRACESWO  | SPI3_SCK       | I2S3_CK        | TIM2_CH2       | SPI1_SCK
{GPIOB, TIMER3, 1,  NULL, ADCx}, // PB4  | 90 | NJTRST     | SPI3_MISO | TIM3_CH1       | SPI1_MISO      | I2S3ext_SD
{GPIOB, TIMER3, 2,  NULL, ADCx}, // PB5  | 91 | I2C1_SMBA  | CAN2_RX   | OTG_HS_ULPI_D7 | ETH_PPS_OUT    | TIM3_CH2       | SPI1_MOSI      | SPI3_MOSI      | DCMI_D10      | I2S3_SD
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB6  | 92 | I2C1_SCL   | TIM4_CH1  | CAN2_TX        | DCMI_D5        | USART1_TX
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB7  | 93 | I2C1_SDA   | FSMC_NL   | DCMI_VSYNC     | USART1_RX      | TIM4_CH2
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB8  | 95 | TIM4_CH3   | SDIO_D4   | TIM10_CH1      | DCMI_D6        | ETH_MII_TXD3   | I2C1_SCL       | CAN1_RX
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB9  | 96 | SPI2_NSS   | I2S2_WS   | TIM4_CH4       | TIM11_CH1      | SDIO_D5        | DCMI_D7        | I2C1_SDA       | CAN1_TX
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB10 | 47 | SPI2_SCK   | I2S2_CK   | I2C2_SCL       | USART3_TX      | OTG_HS_ULPI_D3 | ETH_MII_RX_ER  | TIM2_CH3
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB11 | 48 | I2C2_SDA   | USART3_RX | OTG_HS_ULPI_D4 | ETH_RMII_TX_EN | ETH_MII_TX_EN  | TIM2_CH4
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB12 | 51 | SPI2_NSS   | I2S2_WS   | I2C2_SMBA      | USART3_CK      | TIM1_BKIN      | CAN2_RX        | OTG_HS_ULPI_D5 | ETH_RMII_TXD0 | ETH_MII_TXD0 | OTG_HS_ID
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB13 | 52 | SPI2_SCK   | I2S2_CK   | USART3_CTS     | TIM1_CH1N      | CAN2_TX        | OTG_HS_ULPI_D6 | ETH_RMII_TXD1  | ETH_MII_TXD1
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB14 | 53 | SPI2_MISO  | TIM1_CH2N | TIM12_CH1      | OTG_HS_DM      | USART3_RTS     | TIM8_CH2N      | I2S2ext_SD
{GPIOB,   NULL, 0,  NULL, ADCx}, // PB15 | 54 | SPI2_MOSI  | I2S2_SD   | TIM1_CH3N      | TIM8_CH3N      | TIM12_CH2      | OTG_HS_DP

{GPIOC,   NULL, 0, ADC_1,   10}, // PC0  | 15 | OTG_HS_ULPI_STP                                                  | ADC123_IN10
{GPIOC,   NULL, 0, ADC_1,   11}, // PC1  | 16 | ETH_MDC                                                          | ADC123_IN11
{GPIOC,   NULL, 0, ADC_1,   12}, // PC2  | 17 | SPI2_MISO  | OTG_HS_ULPI_DIR  | ETH_MII_TXD2    | I2S2ext_SD     | ADC123_IN12
{GPIOC,   NULL, 0, ADC_1,   13}, // PC3  | 18 | SPI2_MOSI  | I2S2_SD          | OTG_HS_ULPI_NXT | ETH_MII_TX_CLK | ADC123_IN13
{GPIOC,   NULL, 0, ADC_1,   14}, // PC4  | 33 | ETH_RMII_RX_D0                | ETH_MII_RX_D0                    | ADC12_IN14
{GPIOC,   NULL, 0, ADC_1,   15}, // PC5  | 34 | ETH_RMII_RX_D1                | ETH_MII_RX_D1                    | ADC12_IN15
{GPIOC, TIMER8, 1,  NULL, ADCx}, // PC6  | 63 | I2S2_MCK   | TIM8_CH1/SDIO_D6 | USART6_TX       | DCMI_D0        | TIM3_CH1 
{GPIOC, TIMER8, 2,  NULL, ADCx}, // PC7  | 64 | I2S3_MCK   | TIM8_CH2/SDIO_D7 | USART6_RX       | DCMI_D1        | TIM3_CH2
{GPIOC, TIMER8, 3,  NULL, ADCx}, // PC8  | 65 | TIM8_CH3   | SDIO_D0          | TIM3_CH3        | USART6_CK      | DCMI_D2
{GPIOC, TIMER8, 4,  NULL, ADCx}, // PC9  | 66 | I2S_CKIN   | MCO2             | TIM8_CH4        | SDIO_D1        | I2C3_SDA  | DCMI_D3    | TIM3_CH4
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC10 | 78 | SPI3_SCK   | I2S3_CK          | UART4_TX        | SDIO_D2        | DCMI_D8   | USART3_TX
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC11 | 79 | UART4_RX   | SPI3_MISO        | SDIO_D3         | DCMI_D4        | USART3_RX | I2S3ext_SD
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC12 | 80 | UART5_TX   | SDIO_CK          | DCMI_D9         | SPI3_MOSI      | I2S3_SD   | USART3_CK
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC13 |  7 | RTC_OUT, RTC_TAMP1, RTC_TS
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC14 |  8 | OSC32_IN
{GPIOC,   NULL, 0,  NULL, ADCx}, // PC15 |  9 | OSC32_OUT

{GPIOD,   NULL, 0,  NULL, ADCx}, // PD0  | 81 | FSMC_D2    | CAN1_RX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD1  | 82 | FSMC_D3    | CAN1_TX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD2  | 83 | TIM3_ETR   | UART5_RX   | SDIO_CMD | DCMI_D11
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD3  | 84 | FSMC_CLK   | USART2_CTS
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD4  | 85 | FSMC_NOE   | USART2_RTS
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD5  | 86 | FSMC_NWE   | USART2_TX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD6  | 87 | FSMC_NWAIT | USART2_RX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD7  | 88 | USART2_CK  | FSMC_NE1   | FSMC_NCE2
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD8  | 55 | FSMC_D13   | USART3_TX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD9  | 56 | FSMC_D14   | USART3_RX
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD10 | 57 | FSMC_D15   | USART3_CK
{GPIOD,   NULL, 0,  NULL, ADCx}, // PD11 | 58 | FSMC_CLE   | FSMC_A16   | USART3_CTS
{GPIOD, TIMER4, 1,  NULL, ADCx}, // PD12 | 59 | FSMC_ALE   | FSMC_A17   | TIM4_CH1   | USART3_RTS // remap in
{GPIOD, TIMER4, 2,  NULL, ADCx}, // PD13 | 60 | FSMC_A18                | TIM4_CH2                // remap in
{GPIOD, TIMER4, 3,  NULL, ADCx}, // PD14 | 61 | FSMC_D0                 | TIM4_CH3                // remap in
{GPIOD, TIMER4, 4,  NULL, ADCx}, // PD15 | 62 | FSMC_D1                 | TIM4_CH4                // remap in

{GPIOE,   NULL, 0,  NULL, ADCx}, // PE0  | 97 | TIM4_ETR  | FSMC_NBL0 | DCMI_D2
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE1  | 98 | FSMC_NBL1 | DCMI_D3
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE2  |  1 | TRACECLK  | FSMC_A23  | ETH_MII_TXD3
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE3  |  2 | TRACED0   | FSMC_A19
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE4  |  3 | TRACED1   | FSMC_A20  | DCMI_D4
{GPIOE, TIMER9, 1,  NULL, ADCx}, // PE5  |  4 | TRACED2   | FSMC_A21  | TIM9_CH1     / DCMI_D6
{GPIOE, TIMER9, 2,  NULL, ADCx}, // PE6  |  5 | TRACED3   | FSMC_A22  | TIM9_CH2     / DCMI_D7
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE7  | 38 | FSMC_D4   | TIM1_ETR
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE8  | 39 | FSMC_D5   | TIM1_CH1N
{GPIOE, TIMER1, 1,  NULL, ADCx}, // PE9  | 40 | FSMC_D6   | TIM1_CH1   // remap in
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE10 | 41 | FSMC_D7   | TIM1_CH2N
{GPIOE, TIMER1, 2,  NULL, ADCx}, // PE11 | 42 | FSMC_D8   | TIM1_CH2   // remap in
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE12 | 43 | FSMC_D9   | TIM1_CH3N
{GPIOE, TIMER1, 3,  NULL, ADCx}, // PE13 | 44 | FSMC_D10  | TIM1_CH3   // remap in
{GPIOE, TIMER1, 4,  NULL, ADCx}, // PE14 | 45 | FSMC_D11  | TIM1_CH4   // remap in
{GPIOE,   NULL, 0,  NULL, ADCx}, // PE15 | 46 | FSMC_D12  | TIM1_BKIN
#if 0 // LQFP144 package
{GPIOF,    NULL, 0, NULL, ADCx}, // PF0
{GPIOF,    NULL, 0, NULL, ADCx}, // PF1
{GPIOF,    NULL, 0, NULL, ADCx}, // PF2
{GPIOF,    NULL, 0, NULL, ADCx}, // PF3
{GPIOF,    NULL, 0, NULL, ADCx}, // PF4
{GPIOF,    NULL, 0, NULL, ADCx}, // PF5
{GPIOF, TIMER10, 1, NULL, ADCx}, // PF6
{GPIOF, TIMER11, 1, NULL, ADCx}, // PF7
{GPIOF, TIMER13, 1, NULL, ADCx}, // PF8
{GPIOF, TIMER14, 1, NULL, ADCx}, // PF9
{GPIOF,    NULL, 0, NULL, ADCx}, // PF10
{GPIOF,    NULL, 0, NULL, ADCx}, // PF11
{GPIOF,    NULL, 0, NULL, ADCx}, // PF12
{GPIOF,    NULL, 0, NULL, ADCx}, // PF13
{GPIOF,    NULL, 0, NULL, ADCx}, // PF14
{GPIOF,    NULL, 0, NULL, ADCx}, // PF15

{GPIOG,    NULL, 0, NULL, ADCx}, // PG0
{GPIOG,    NULL, 0, NULL, ADCx}, // PG1
{GPIOG,    NULL, 0, NULL, ADCx}, // PG2
{GPIOG,    NULL, 0, NULL, ADCx}, // PG3
{GPIOG,    NULL, 0, NULL, ADCx}, // PG4
{GPIOG,    NULL, 0, NULL, ADCx}, // PG5
{GPIOG,    NULL, 0, NULL, ADCx}, // PG6
{GPIOG,    NULL, 0, NULL, ADCx}, // PG7 
{GPIOG,    NULL, 0, NULL, ADCx}, // PG8
{GPIOG,    NULL, 0, NULL, ADCx}, // PG9
{GPIOG,    NULL, 0, NULL, ADCx}, // PG10
{GPIOG,    NULL, 0, NULL, ADCx}, // PG11
{GPIOG,    NULL, 0, NULL, ADCx}, // PG12
{GPIOG,    NULL, 0, NULL, ADCx}, // PG13
{GPIOG,    NULL, 0, NULL, ADCx}, // PG14
{GPIOG,    NULL, 0, NULL, ADCx}  // PG15
#endif
};

/* to be defined
extern const uint8 boardPWMPins[BOARD_NR_PWM_PINS] __FLASH__ = {
    0, 1, 2, 3, 15, 16, 17, 19, 20, 21, 38, 39, 49, 41, 60, 61, 62, 63, 73, 75, 77, 78
};
*/
const uint8 boardADCPins[BOARD_NR_ADC_PINS] = {
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PC0, PC1, PC2, PC3, PC4, PC5
};

const uint8 boardUsedPins[BOARD_NR_USED_PINS] = {
    BOARD_LED_PIN, BOARD_LED2_PIN, BOARD_BUTTON1_PIN, BOARD_BUTTON2_PIN, BOARD_BUTTON2_PIN,
	BOARD_JTMS_SWDIO_PIN, BOARD_JTCK_SWCLK_PIN,
	FLASH_CS_PIN, FLASH_CLK_PIN, FLASH_DO_PIN, FLASH_DI_PIN,
	NRF24_CE_PIN, NRF24_CS_PIN, NRF24_IRQ_PIN,
	BOARD_SDIO_D0, BOARD_SDIO_D1, BOARD_SDIO_D2, BOARD_SDIO_D3, BOARD_SDIO_CLK, BOARD_SDIO_CMD,
	USB_DM_PIN, USB_DP_PIN
};

#ifdef __cplusplus
}
#endif
