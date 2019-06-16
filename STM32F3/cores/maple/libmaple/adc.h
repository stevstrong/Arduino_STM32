/******************************************************************************
 * The MIT License
 *
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
 * @file libmaple/stm32f3/include/series/adc.h
 * @author F3-port by Hanspeter Portner <dev@open-music-kontrollers.ch>
 * @brief STM32F3 ADC support.
 */

#ifndef _LIBMAPLE_STM32F3_ADC_H_
#define _LIBMAPLE_STM32F3_ADC_H_

#include <libmaple/libmaple.h>
#include <libmaple/bitband.h>
#include <libmaple/libmaple_types.h>
#include <libmaple/rcc.h>       /* For the prescalers */


#ifdef __cplusplus
extern "C"{
#endif



/**
 * TODO document
 */
typedef struct adc_callback_data {
	uint32 irq_flags;	
	void *arg;
} adc_callback_data;

/**
 * TODO document
 */
typedef struct adc_private_data {
    void (*handler)(adc_callback_data*);
    uint32 handler_flags;
    adc_callback_data cb_data;
} adc_private_data;

/*
 * Register map
 */

/*
 * ADC individual register map type.
 */
typedef struct adc_reg_map {
	__IO uint32 ISR;				/**< */
	__IO uint32 IER;				/**< */
	__IO uint32 CR;					/**< */
	__IO uint32 CFGR;				/**< */
	uint32 reserved1;	
	__IO uint32 SMPR1;			/**< */
	__IO uint32 SMPR2;			/**< */
	uint32 reserved2;	
	__IO uint32 TR1;				/**< */
	__IO uint32 TR2;				/**< */
	__IO uint32 TR3;				/**< */
	uint32 reserved3;	
	__IO uint32 SQR1;				/**< */
	__IO uint32 SQR2;				/**< */
	__IO uint32 SQR3;				/**< */
	__IO uint32 SQR4;				/**< */
	__IO uint32 DR;					/**< */
	uint32 reserved4 [2];
	__IO uint32 JSQR;				/**< */
	uint32 reserved5 [4];
	__IO uint32 OFR1;				/**< */
	__IO uint32 OFR2;				/**< */
	__IO uint32 OFR3;				/**< */
	__IO uint32 OFR4;				/**< */
	uint32 reserved6 [4];
	__IO uint32 JDR1;				/**< */
	__IO uint32 JDR2;				/**< */
	__IO uint32 JDR3;				/**< */
	__IO uint32 JDR4;				/**< */
	uint32 reserved7 [4];
	__IO uint32 AWD2CR;			/**< */
	__IO uint32 AWD3CR;			/**< */
	uint32 reserved8 [2];
	__IO uint32 DIFSEL;			/**< */
	__IO uint32 CALFACT;		/**< */
} adc_reg_map;

/*
 * ADC master and slave common register map type.
 */
typedef struct adc_common_reg_map {
	__IO uint32 CSR;				/**< */
	uint32 reserved;
	__IO uint32 CCR;				/**< */
	__IO uint32 CDR;				/**< */
} adc_common_reg_map;

/** ADC device type. */
typedef struct adc_dev {
    adc_reg_map *regs; /**< Register map */
    rcc_clk_id clk_id; /**< RCC clock information */
		adc_private_data *priv;	/**< ADC private data */
} adc_dev;

typedef struct adc_info {
	const adc_dev * device;
	uint8 channel;
} adc_info;

/*
 * Devices
 */

extern const adc_dev adc1;	/* master */
extern const adc_dev adc2;	/* slave */
extern const adc_dev adc3;	/* master */
extern const adc_dev adc4;	/* slave */
#define ADC1 (&adc1)
#define ADC2 (&adc2)
#define ADC3 (&adc3)
#define ADC4 (&adc4)

/*
 * Register map base pointers
 */

/** STM32F3 ADC1 individual register map base pointer. */
#define ADC1_BASE                       ((struct adc_reg_map*)0x50000000)
/** STM32F3 ADC2 individual register map base pointer. */
#define ADC2_BASE                       ((struct adc_reg_map*)0x50000100)
/** STM32F3 ADC1/ADC2 common register map base pointer. */
#define ADC12_BASE											((struct adc_common_reg_map*)0x50000300)
/** STM32F3 ADC3 individual register map base pointer. */
#define ADC3_BASE                       ((struct adc_reg_map*)0x50000400)
/** STM32F3 ADC4 individual register map base pointer. */
#define ADC4_BASE                       ((struct adc_reg_map*)0x50000500)
/** STM32F3 ADC3/ADC4 common register map base pointer. */
#define ADC34_BASE											((struct adc_common_reg_map*)0x50000700)

/*
 * Register bit definitions
 */

/* Interrupt and status register */
#define ADC_ISR_JQOVF_BIT					10
#define ADC_ISR_AWD3_BIT					9
#define ADC_ISR_AWD2_BIT					8
#define ADC_ISR_AWD1_BIT					7
#define ADC_ISR_JEOS_BIT					6
#define ADC_ISR_JEOC_BIT					5
#define ADC_ISR_OVR_BIT						4
#define ADC_ISR_EOS_BIT						3
#define ADC_ISR_EOC_BIT						2
#define ADC_ISR_EOSMP_BIT					1
#define ADC_ISR_ADRDY_BIT					0

#define ADC_ISR_JQOVF							(1U << ADC_ISR_JQOVF_BIT)
#define ADC_ISR_AWD3							(1U << ADC_ISR_AWD3_BIT)
#define ADC_ISR_AWD2          		(1U << ADC_ISR_AWD2_BIT)
#define ADC_ISR_AWD1          		(1U << ADC_ISR_AWD1_BIT)
#define ADC_ISR_JEOS          		(1U << ADC_ISR_JEOS_BIT)
#define ADC_ISR_JEOC          		(1U << ADC_ISR_JEOC_BIT)
#define ADC_ISR_OVR           		(1U << ADC_ISR_OVR_BIT)
#define ADC_ISR_EOS           		(1U << ADC_ISR_EOS_BIT)
#define ADC_ISR_EOC           		(1U << ADC_ISR_EOC_BIT)
#define ADC_ISR_EOSMP         		(1U << ADC_ISR_EOSMP_BIT)
#define ADC_ISR_ADRDY         		(1U << ADC_ISR_ADRDY_BIT)

/* FIXME make an enum out of this */
#define ADC_WATCHDOG_1_INTERRUPT				ADC_ISR_AWD1
#define ADC_WATCHDOG_2_INTERRUPT				ADC_ISR_AWD2
#define ADC_WATCHDOG_3_INTERRUPT				ADC_ISR_AWD3
#define ADC_CONV_INTERRUPT							ADC_ISR_EOC
#define ADC_INJ_CONV_INTERRUPT					ADC_ISR_JEOC
#define ADC_OVERRUN_INTERRUPT						ADC_ISR_OVR
#define ADC_ALL_INTERRUPTS							(ADC_CONV_INTERRUPT | ADC_WATCHDOG_1_INTERRUPT | ADC_WATCHDOG_2_INTERRUPT | ADC_WATCHDOG_3_INTERRUPT | ADC_INJ_CONV_INTERRUPT | ADC_OVERRUN_INTERRUPT)

/* Interrupt enable register */
#define ADC_IER_JQOVF_BIT					10
#define ADC_IER_AWD3_BIT					9
#define ADC_IER_AWD2_BIT					8
#define ADC_IER_AWD1_BIT					7
#define ADC_IER_JEOS_BIT					6
#define ADC_IER_JEOC_BIT					5
#define ADC_IER_OVR_BIT						4
#define ADC_IER_EOS_BIT						3
#define ADC_IER_EOC_BIT						2
#define ADC_IER_EOSMP_BIT					1
#define ADC_IER_ADRDY_BIT					0

#define ADC_IER_JQOVF							(1U << ADC_IER_JQOVF_BIT)
#define ADC_IER_AWD3							(1U << ADC_IER_AWD3_BIT)
#define ADC_IER_AWD2          		(1U << ADC_IER_AWD2_BIT)
#define ADC_IER_AWD1          		(1U << ADC_IER_AWD1_BIT)
#define ADC_IER_JEOS          		(1U << ADC_IER_JEOS_BIT)
#define ADC_IER_JEOC          		(1U << ADC_IER_JEOC_BIT)
#define ADC_IER_OVR           		(1U << ADC_IER_OVR_BIT)
#define ADC_IER_EOS           		(1U << ADC_IER_EOS_BIT)
#define ADC_IER_EOC           		(1U << ADC_IER_EOC_BIT)
#define ADC_IER_EOSMP         		(1U << ADC_IER_EOSMP_BIT)
#define ADC_IER_ADRDY         		(1U << ADC_IER_ADRDY_BIT)

/* Control register */
#define ADC_CR_ADCAL_BIT					31
#define ADC_CR_ADCALDIF_BIT				30
#define ADC_CR_ADVREGEN_SHIFT			28
#define ADC_CR_JADSTP_BIT					5
#define ADC_CR_ADSTP_BIT					4
#define ADC_CR_JADSTART_BIT				3
#define ADC_CR_ADSTART_BIT				2
#define ADC_CR_ADDIS_BIT					1
#define ADC_CR_ADEN_BIT						0

#define ADC_CR_ADCAL							(1U << ADC_CR_ADCAL_BIT)
#define ADC_CR_ADCALDIF       		(1U << ADC_CR_ADCALDIF_BIT)
#define ADC_CR_ADVREGEN						(0x3 << ADC_CR_ADVREGEN_SHIFT)
#define ADC_CR_ADVREGEN_ENABLE		(0x1 << ADC_CR_ADVREGEN_SHIFT)
#define ADC_CR_ADVREGEN_DISABLE		(0x2 << ADC_CR_ADVREGEN_SHIFT)
#define ADC_CR_JADSTP         		(1U << ADC_CR_JADSTP_BIT)
#define ADC_CR_ADSTP          		(1U << ADC_CR_ADSTP_BIT)
#define ADC_CR_JADSTART       		(1U << ADC_CR_JADSTART_BIT)
#define ADC_CR_ADSTART        		(1U << ADC_CR_ADSTART_BIT)
#define ADC_CR_ADDIS          		(1U << ADC_CR_ADDIS_BIT)
#define ADC_CR_ADEN           		(1U << ADC_CR_ADEN_BIT)

/* Configuration register */
#define ADC_CFGR_AWD1CH_SHIFT			26
#define ADC_CFGR_JAUTO_BIT				25
#define ADC_CFGR_JAWD1EN_BIT			24
#define ADC_CFGR_AWD1EN_BIT				23
#define ADC_CFGR_AWD1SGL_BIT			22
#define ADC_CFGR_JQM_BIT					21
#define ADC_CFGR_JDISCEN_BIT			20
#define ADC_CFGR_JDISCNUM_SHIFT		17
#define ADC_CFGR_DISCEN_BIT				16
#define ADC_CFGR_AUTDLY_BIT				14
#define ADC_CFGR_CONT_BIT					13
#define ADC_CFGR_OVRMOD_BIT				12
#define ADC_CFGR_EXTEN_SHIFT			10
#define ADC_CFGR_EXTSEL_SHIFT			6
#define ADC_CFGR_ALIGN_BIT				5
#define ADC_CFGR_RES_SHIFT				3
#define ADC_CFGR_DMACFG_BIT				1
#define ADC_CFGR_DMAEN_BIT				0

#define ADC_CFGR_AWD1CH						(0x1F << ADC_CFGR_AWD1CH_SHIFT)
#define ADC_CFGR_JAUTO						(1U << ADC_CFGR_JAUTO_BIT)
#define ADC_CFGR_JAWD1EN					(1U << ADC_CFGR_JAWD1EN_BIT)
#define ADC_CFGR_AWD1EN						(1U << ADC_CFGR_AWD1EN_BIT)
#define ADC_CFGR_AWD1SGL					(1U << ADC_CFGR_AWD1SGL_BIT)
#define ADC_CFGR_JQM							(1U << ADC_CFGR_JQM_BIT)
#define ADC_CFGR_JDISCEN					(1U << ADC_CFGR_JDISCEN_BIT)
#define ADC_CFGR_JDISCNUM					(0x7 << ADC_CFGR_JDISCNUM_SHIFT)
#define ADC_CFGR_DISCEN						(1U << ADC_CFGR_DISCEN_BIT)
#define ADC_CFGR_AUTODLY					(1U << ADC_CFGR_AUTODLY_BIT)
#define ADC_CFGR_CONT							(1U << ADC_CFGR_CONT_BIT)
#define ADC_CFGR_OVRMOD						(1U << ADC_CFGR_OVRMOD_BIT)
#define ADC_CFGR_EXTEN						(0x3 << ADC_CFGR_EXTEN_SHIFT)
#define ADC_CFGR_EXTSEL						(0xF << ADC_CFGR_EXTSEL_SHIFT)
#define ADC_CFGR_ALIGN						(1U << ADC_CFGR_ALIGN_BIT)
#define ADC_CFGR_RES							(0x3 << ADC_CFGR_RES_SHIFT)
#define ADC_CFGR_DMACFG						(1U << ADC_CFGR_DMACFG_BIT)
#define ADC_CFGR_DMAEN						(1U << ADC_CFGR_DMAEN_BIT)

/* Sample time register 1 */
#define ADC_SMPR1_SMP9                  (0x7 << 27)
#define ADC_SMPR1_SMP8                  (0x7 << 24)
#define ADC_SMPR1_SMP7                  (0x7 << 21)
#define ADC_SMPR1_SMP6                  (0x7 << 18)
#define ADC_SMPR1_SMP5                  (0x7 << 15)
#define ADC_SMPR1_SMP4                  (0x7 << 12)
#define ADC_SMPR1_SMP3                  (0x7 << 9)
#define ADC_SMPR1_SMP2                  (0x7 << 6)
#define ADC_SMPR1_SMP1                  (0x7 << 3)

/* Sample time register 2 */
#define ADC_SMPR2_SMP18                 (0x7 << 24)
#define ADC_SMPR2_SMP17                 (0x7 << 21)
#define ADC_SMPR2_SMP16                 (0x7 << 18)
#define ADC_SMPR2_SMP15                 (0x7 << 15)
#define ADC_SMPR2_SMP14                 (0x7 << 12)
#define ADC_SMPR2_SMP13                 (0x7 << 9)
#define ADC_SMPR2_SMP12                 (0x7 << 6)
#define ADC_SMPR2_SMP11                 (0x7 << 3)
#define ADC_SMPR2_SMP10                 0x7

/* Analog watchdog 1 threshold register */
#define ADC_TR1_HT1											(0xFFF << 16) /* 12bit watchdog */
#define ADC_TR1_LT1											0xFFF

/* Analog watchdog 2 threshold register */
#define ADC_TR2_HT2											(0xFF << 16)	/* 8bit watchdog */
#define ADC_TR2_LT2											0xFF

/* Analog watchdog 3 threshold register */
#define ADC_TR3_HT3											(0xFF << 16)	/* 8bit watchdog */
#define ADC_TR3_LT3											0xFF

/* Regular sequence register 1 */
#define ADC_SQR1_SQ4                   	(0x1F << 24)
#define ADC_SQR1_SQ3                   	(0x1F << 18)
#define ADC_SQR1_SQ2                   	(0x1F << 12)
#define ADC_SQR1_SQ1                   	(0x1F << 6)
#define ADC_SQR1_L                      0xF

/* Regular sequence register 2 */
#define ADC_SQR2_SQ9                   	(0x1F << 24)
#define ADC_SQR2_SQ8                   	(0x1F << 18)
#define ADC_SQR2_SQ7                   	(0x1F << 12)
#define ADC_SQR2_SQ6                   	(0x1F << 6)
#define ADC_SQR2_SQ5                    0x1F

/* Regular sequence register 3 */
#define ADC_SQR3_SQ14                  	(0x1F << 24)
#define ADC_SQR3_SQ13                  	(0x1F << 18)
#define ADC_SQR3_SQ12                  	(0x1F << 12)
#define ADC_SQR3_SQ11                  	(0x1F << 6)
#define ADC_SQR3_SQ10                   0x1F

/* Regular sequence register 4 */
#define ADC_SQR4_SQ16                  	(0x1F << 6)
#define ADC_SQR4_SQ15                   0x1F

/* Regular data register */
#define ADC_DR_RDATA										0xFFFF

/* Injected sequence register */
#define ADC_JSQR_JEXTEN_SHIFT						6
#define ADC_JSQR_JEXTSEL_SHIFT					2

#define ADC_JSQR_SQ4										(0x1F << 26)
#define ADC_JSQR_SQ3										(0x1F << 20)
#define ADC_JSQR_SQ2										(0x1F << 14)
#define ADC_JSQR_SQ1										(0x1F << 8)
#define ADC_JSQR_JEXTEN									(0x3 << ADC_JSQR_JEXTEN_SHIFT)
#define ADC_JSQR_JEXTSEL								(0xF << ADC_JSQR_JEXTSEL_SHIFT)
#define ADC_JSQR_JL											0x3

/* Offset register 1 */
#define ADC_OFR1_OFFSET1EN_BIT					31

#define ADC_OFR1_OFFSET1ENT							(1U << ADC_OFR1_OFFSET1EN_BIT)
#define ADC_OFR1_OFFSET1_CH							(0x1F << 26)
#define ADC_OFR1_OFFSET1								0xFFF

/* Offset register 2 */
#define ADC_OFR2_OFFSET2EN_BIT					31

#define ADC_OFR2_OFFSET2EN							(1U << ADC_OFR2_OFFSET2EN_BIT)
#define ADC_OFR2_OFFSET2_CH							(0x1F << 26)
#define ADC_OFR2_OFFSET2								0xFFF

/* Offset register 3 */
#define ADC_OFR3_OFFSET3EN_BIT					31

#define ADC_OFR3_OFFSET3EN							(1U << ADC_OFR3_OFFSET3EN_BIT)
#define ADC_OFR3_OFFSET3_CH							(0x1F << 26)
#define ADC_OFR3_OFFSET3								0xFFF

/* Offset register 4 */
#define ADC_OFR4_OFFSET4EN_BIT					31

#define ADC_OFR4_OFFSET4EN							(1U << ADC_OFR4_OFFSET4EN_BIT)
#define ADC_OFR4_OFFSET4_CH							(0x1F << 26)
#define ADC_OFR4_OFFSET4								0xFFF

/* Injected data register 1 */
#define ADC_JDR1_JDATA1									0xFFFF

/* Injected data register 2 */
#define ADC_JDR2_JDATA2									0xFFFF

/* Injected data register 3 */
#define ADC_JDR3_JDATA3									0xFFFF

/* Injected data register 4 */
#define ADC_JDR4_JDATA4									0xFFFF

/* Analog watchdog 2 configuration register */
#define ADC_AWD2CR_AWD2CH								(0x1FFFF << 1)

/* Analog watchdog 3 configuration register */
#define ADC_AWD3CR_AWD3CH								(0x1FFFF << 1)

/* Differential mode selection register */
#define ADC_DIFSEL_DIFSEL								(0x1FFFF << 1)

/* Calibration factors */
#define ADC_CALFACT_CALFACT_D						(0x7F << 16)
#define ADC_CALFACT_CALFACT_S						0x7F

/* Common status register */
#define ADC_CSR_JQOVF_SLV_BIT						26
#define ADC_CSR_AWD3_SLV_BIT						25
#define ADC_CSR_AWD2_SLV_BIT						24
#define ADC_CSR_AWD1_SLV_BIT						23
#define ADC_CSR_JEOS_SLV_BIT						22
#define ADC_CSR_JEOC_SLV_BIT						21
#define ADC_CSR_OVR_SLV_BIT							20
#define ADC_CSR_EOS_SLV_BIT							19
#define ADC_CSR_EOC_SLV_BIT							18
#define ADC_CSR_EOSMP_SLV_BIT						17
#define ADC_CSR_ADRDY_SLV_BIT						16

#define ADC_CSR_JQOVF_MST_BIT						10
#define ADC_CSR_AWD3_MST_BIT						9
#define ADC_CSR_AWD2_MST_BIT						8
#define ADC_CSR_AWD1_MST_BIT						7
#define ADC_CSR_JEOS_MST_BIT						6
#define ADC_CSR_JEOC_MST_BIT						5
#define ADC_CSR_OVR_MST_BIT							4
#define ADC_CSR_EOS_MST_BIT							3
#define ADC_CSR_EOC_MST_BIT							2
#define ADC_CSR_EOSMP_MST_BIT						1
#define ADC_CSR_ADRDY_MST_BIT						0

#define ADC_CSR_JQOVF_SLV								(1U << ADC_CSR_JQOVF_SLV_BIT)
#define ADC_CSR_AWD3_SLV                (1U << ADC_CSR_AWD3_SLV_BIT)
#define ADC_CSR_AWD2_SLV                (1U << ADC_CSR_AWD2_SLV_BIT)
#define ADC_CSR_AWD1_SLV                (1U << ADC_CSR_AWD1_SLV_BIT)
#define ADC_CSR_JEOS_SLV                (1U << ADC_CSR_JEOS_SLV_BIT)
#define ADC_CSR_JEOC_SLV                (1U << ADC_CSR_JEOC_SLV_BIT)
#define ADC_CSR_OVR_SLV                 (1U << ADC_CSR_OVR_SLV_BIT)
#define ADC_CSR_EOS_SLV                 (1U << ADC_CSR_EOS_SLV_BIT)
#define ADC_CSR_EOC_SLV                 (1U << ADC_CSR_EOC_SLV_BIT)
#define ADC_CSR_EOSMP_SLV              	(1U << ADC_CSR_EOS_MP_SLV_BIT)
#define ADC_CSR_ADRDY_SLV               (1U << ADC_CSR_ADRDY_SLV_BIT)

#define ADC_CSR_JQOVF_MST								(1U << ADC_CSR_JQOVF_MST_BIT)
#define ADC_CSR_AWD3_MST                (1U << ADC_CSR_AWD3_MST_BIT)
#define ADC_CSR_AWD2_MST                (1U << ADC_CSR_AWD2_MST_BIT)
#define ADC_CSR_AWD1_MST                (1U << ADC_CSR_AWD1_MST_BIT)
#define ADC_CSR_JEOS_MST                (1U << ADC_CSR_JEOS_MST_BIT)
#define ADC_CSR_JEOC_MST                (1U << ADC_CSR_JEOC_MST_BIT)
#define ADC_CSR_OVR_MST                 (1U << ADC_CSR_OVR_MST_BIT)
#define ADC_CSR_EOS_MST                 (1U << ADC_CSR_EOS_MST_BIT)
#define ADC_CSR_EOC_MST                 (1U << ADC_CSR_EOC_MST_BIT)
#define ADC_CSR_EOSMP_MST              	(1U << ADC_CSR_EOS_MP_MST_BIT)
#define ADC_CSR_ADRDY_MST               (1U << ADC_CSR_ADRDY_MST_BIT)

/* Common control register */
#define ADC_CCR_VBATEN_BIT							24
#define ADC_CCR_TSEN_BIT								23
#define ADC_CCR_VREFEN_BIT							22
#define ADC_CCR_CKMODE_SHIFT						16
#define ADC_CCR_MDMA_SHIFT							14
#define ADC_CCR_DMACFG_BIT							13
#define ADC_CCR_DELAY_SHIFT							8
#define ADC_CCR_DUAL_SHIFT							0

#define ADC_CCR_VBATEN									(1U << ADC_CCR_VBATEN_BIT)
#define ADC_CCR_TSEN										(1U << ADC_CCR_TSEN_BIT)
#define ADC_CCR_VREFEN									(1U << ADC_CCR_VREFEN_BIT)
#define ADC_CCR_CKMODE									(0x3 << ADC_CCR_CKMODE_SHIFT)
#define ADC_CCR_MDMA										(0x3 << ADC_CCR_MDMA_SHIFT)
#define ADC_CCR_DMACFG									(1U << ADC_CCR_DMACFG_BIT)
#define ADC_CCR_DELAY										(0xF << ADC_CCR_DELAY_SHIFT)
#define ADC_CCR_DUAL										0X1F

/* Common regular data register for dual mode */
#define ADC_CDR_RDATA_SLV								(0xFFFF << 16)
#define ADC_CDR_RDATA_MST								0xFFFF

/*
 * Other types
 */

/**
 * @brief STM32F3 external event selectors for regular group
 *        conversion.
 *
 * For ease of use, each event selector is given along with the ADCs
 * it's available on, along with any other availability restrictions.
 *
 * @see adc_set_extsel()
 */
typedef enum adc_extsel_event {
    /* ADC1 (master) and ADC2 (slave) regular channels */
    ADC12_EXT_EV_TIM1_CC1  									= 0x0 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM1_CC2  									= 0x1 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM1_CC3  									= 0x2 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM2_CC2  									= 0x3 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM3_TRGO 									= 0x4 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM4_CC4  									= 0x5 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_EXTI_11   									= 0x6 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM8_TRGO 									= 0x7 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM8_TRGO2									= 0x8 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM1_TRGO									= 0x9 << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM1_TRGO2									= 0xA << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM2_TRGO 									= 0xB << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM4_TRGO 									= 0xC << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM6_TRGO 									= 0xD << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM15_TRGO									= 0xE << ADC_CFGR_EXTSEL_SHIFT,
    ADC12_EXT_EV_TIM3_CC4  									= 0xF << ADC_CFGR_EXTSEL_SHIFT,

    /* ADC1 (master) and ADC2 (slave) injected channels */
    ADC12_JEXT_EV_TIM1_TRGO									= 0x0 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM1_CC4									= 0x1 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM2_TRGO									= 0x2 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM2_CC1									= 0x3 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM3_CC4									= 0x4 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM4_TRGO									= 0x5 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_EXTI_15										= 0x6 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM8_CC4									= 0x7 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM1_TRGO2								= 0x8 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM8_TRGO 								= 0x9 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM8_TRGO2								= 0xA << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM3_CC3									= 0xB << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM3_TRGO									= 0xC << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM3_CC1									= 0xD << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM6_TRGO									= 0xE << ADC_JSQR_JEXTSEL_SHIFT,
    ADC12_JEXT_EV_TIM15_TRGO								= 0xF << ADC_JSQR_JEXTSEL_SHIFT,

    /* ADC3 (master) and ADC4 (slave) injected channels */
    ADC34_EXT_EV_TIM3_CC1										= 0x0 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM3_CC3										= 0x1 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM1_cc3										= 0x2 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM8_CC1										= 0x3 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM8_TRGO									= 0x4 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_EXTI_2											= 0x5 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM4_CC1										= 0x6 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM2_TRGO									= 0x7 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM8_TRGO2									= 0x8 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM1_TRGO 									= 0x9 << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM1_TRGO2									= 0xA << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM3_TRGO									= 0xB << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM4_TRGO									= 0xC << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM7_TRGO									= 0xD << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM15_TRGO									= 0xE << ADC_CFGR_EXTSEL_SHIFT,
    ADC34_EXT_EV_TIM2_CC1										= 0xF << ADC_CFGR_EXTSEL_SHIFT,

    /* ADC3 (master) and ADC4 (slave) injected channels */
    ADC34_JEXT_EV_TIM1_TRGO									= 0x0 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM1_CC4									= 0x1 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM4_CC3									= 0x2 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM8_CC2									= 0x3 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM8_CC4									= 0x4 << ADC_JSQR_JEXTSEL_SHIFT,
    /* ADC34_JEXT_EV_TIM4_CC3		= 0x5 << ADC_JSQR_JEXTSEL_SHIFT, */ /* FIXME duplicate or typo in manual? */
    ADC34_JEXT_EV_TIM4_CC4									= 0x6 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM4_TRGO									= 0x7 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM1_TRGO2								= 0x8 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM8_TRGO 								= 0x9 << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM8_TRGO2								= 0xA << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM1_CC3									= 0xB << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM3_TRGO									= 0xC << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM2_TRGO									= 0xD << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM7_TRGO									= 0xE << ADC_JSQR_JEXTSEL_SHIFT,
    ADC34_JEXT_EV_TIM15_TRGO								= 0xF << ADC_JSQR_JEXTSEL_SHIFT,
} adc_extsel_event;

/**
 * @brief STM32F3 ADC trigger mode.
 *
 * These control the trigger mode of the ADC, which can be set to:
 * Software trigger, hardware trigger on rising, falling or both edges.
 */
typedef enum adc_exttrig_mode {
		ADC_EXTTRIG_MODE_SOFTWARE								= 0x0 << ADC_CFGR_EXTEN_SHIFT,		/**< .. */
		ADC_EXTTRIG_MODE_HARDWARE_RISING				= 0x1 << ADC_CFGR_EXTEN_SHIFT,		/**< .. */
		ADC_EXTTRIG_MODE_HARDWARE_FALLING				= 0x2 << ADC_CFGR_EXTEN_SHIFT,		/**< .. */
		ADC_EXTTRIG_MODE_HARDWARE_BOTH					= 0x3 << ADC_CFGR_EXTEN_SHIFT,		/**< .. */
} adc_exttrig_mode;

/**
 * @brief STM32F3 ADC resolution.
 *
 * These control the ADC resolution, which can be set to:
 * 6, 8, 10 or 12 bit.
 */
typedef enum adc_resolution {
		ADC_RESOLUTION_12_BIT										= 0x0 << ADC_CFGR_RES_SHIFT, /**< .. */
		ADC_RESOLUTION_10_BIT										= 0x1 << ADC_CFGR_RES_SHIFT, /**< .. */
		ADC_RESOLUTION_8_BIT										= 0x2 << ADC_CFGR_RES_SHIFT, /**< .. */
		ADC_RESOLUTION_6_BIT										= 0x3 << ADC_CFGR_RES_SHIFT, /**< .. */
} adc_resolution;

/**
 * @brief STM32F3 sample times, in ADC clock cycles.
 *
 * These control the amount of time spent sampling the input voltage.
 */
typedef enum adc_smp_rate {
    ADC_SMPR_1_5  													= 0x0,    /**< 1.5 ADC cycles */
    ADC_SMPR_2_5  													= 0x1,    /**< 2.5 ADC cycles */
    ADC_SMPR_4_5  													= 0x2,    /**< 4.5 ADC cycles */
    ADC_SMPR_7_5  													= 0x3,    /**< 7.5 ADC cycles */
    ADC_SMPR_19_5 													= 0x4,    /**< 19.5 ADC cycles */
    ADC_SMPR_61_5 													= 0x5,    /**< 61.5 ADC cycles */
    ADC_SMPR_181_5													= 0x6,    /**< 181.5 ADC cycles */
    ADC_SMPR_601_5													= 0x7,    /**< 601.5 ADC cycles */
} adc_smp_rate;

/*
 * FIXME SHIFT
 */

/**
 * @brief STM32F3 ADC clock mode.
 */
typedef enum adc_clock_mode {
		ADC_CLOCK_MODE_ASYNCHRONOUS							=	0x0,		/**< . */
		ADC_CLOCK_MODE_SYNCHRONOUS_DIV_1				=	0x1,		/**< . */
		ADC_CLOCK_MODE_SYNCHRONOUS_DIV_2				=	0x2,		/**< . */
		ADC_CLOCK_MODE_SYNCHRONOUS_DIV_4				=	0x3,		/**< . */
} adc_clock_mode;

/**
 * @brief STM32F3 ADC DMA mode.
 */
typedef enum adc_dma_mode {
		ADC_MDMA_MODE_DISABLE										=	(0x0 << ADC_CCR_MDMA_SHIFT), 		/**< . */
		ADC_MDMA_MODE_ENABLE_12_10_BIT					=	(0x2 << ADC_CCR_MDMA_SHIFT), 		/**< . */
		ADC_MDMA_MODE_ENABLE_8_6_BIT						=	(0x3 << ADC_CCR_MDMA_SHIFT), 		/**< . */
} adc_dma_mode;

/**
 * @brief STM32F3 ADC mode.
 */
typedef enum adc_mode {
		ADC_MODE_INDEPENDENT										=	0x0, 		/**< . */
		ADC_MODE_DUAL_REGULAR_INJECTED					=	0x1, 		/**< . */
		ADC_MODE_DUAL_REGULAR_ALTERNATE_TRIGGER	=	0x2, 		/**< . */
		ADC_MODE_DUAL_INJECTED_ONLY							=	0x5, 		/**< . */
		ADC_MODE_DUAL_REGULAR_ONLY							=	0x6, 		/**< . */
		ADC_MODE_DUAL_INTERLEAVED_ONLY					=	0x7, 		/**< . */
		ADC_MODE_DUAL_ALTERNATE_TRIGGER_ONLY		=	0x9, 		/**< . */
} adc_mode;

/**
 * @brief STM32F3 ADC prescalers, as divisors of PCLK2.
 */
typedef enum adc_prescaler {
    /** ADC clock disabled, AHB clock enabled */
    ADC_PRE_AHBCLK_DIV_1 	= ADC_CLOCK_MODE_SYNCHRONOUS_DIV_1,
    ADC_PRE_AHBCLK_DIV_2 	= ADC_CLOCK_MODE_SYNCHRONOUS_DIV_2,
    ADC_PRE_AHBCLK_DIV_4 	= ADC_CLOCK_MODE_SYNCHRONOUS_DIV_4,

		/** ADC clock enabled, AHB clock disabled */
    ADC_PRE_PCLK_DIV_1 		= RCC_ADCPRE_PCLK_DIV_1,		/** PCLK divided by 1 */
    ADC_PRE_PCLK_DIV_2 		= RCC_ADCPRE_PCLK_DIV_2,		/** PCLK divided by 2 */
    ADC_PRE_PCLK_DIV_4 		= RCC_ADCPRE_PCLK_DIV_4,		/** PCLK divided by 4 */
    ADC_PRE_PCLK_DIV_6 		= RCC_ADCPRE_PCLK_DIV_6,		/** PCLK divided by 6 */
    ADC_PRE_PCLK_DIV_8 		= RCC_ADCPRE_PCLK_DIV_8,		/** PCLK divided by 8 */
    ADC_PRE_PCLK_DIV_10 	= RCC_ADCPRE_PCLK_DIV_10,		/** PCLK divided by 10 */
    ADC_PRE_PCLK_DIV_12 	= RCC_ADCPRE_PCLK_DIV_12,		/** PCLK divided by 12 */
    ADC_PRE_PCLK_DIV_16 	= RCC_ADCPRE_PCLK_DIV_16,		/** PCLK divided by 16 */
    ADC_PRE_PCLK_DIV_32 	= RCC_ADCPRE_PCLK_DIV_32,		/** PCLK divided by 32 */
    ADC_PRE_PCLK_DIV_64 	= RCC_ADCPRE_PCLK_DIV_64,		/** PCLK divided by 64 */
    ADC_PRE_PCLK_DIV_128 	= RCC_ADCPRE_PCLK_DIV_128,	/** PCLK divided by 128 */
    ADC_PRE_PCLK_DIV_256 	= RCC_ADCPRE_PCLK_DIV_256,	/** PCLK divided by 256 */
} adc_prescaler;

/*
 * Routines
 */

/**
 * @brief Calibrate an ADC peripheral
 *
 * Availability: STM32F1, STM32F3.
 *
 * @param dev adc device
 */
void adc_calibrate(const adc_dev *dev);

/**
 * @brief Set external trigger conversion mode event for regular channels
 *
 * Availability: STM32F1, STM32F3.
 *
 * @param dev    ADC device
 * @param mode	 external trigger disabled, on rising edge, on falling edge or on both
 */
void adc_set_exttrig(const adc_dev *dev, adc_exttrig_mode mode);

/**
 * @brief Set ADC resolution of an ADC peripheral
 *
 * Availability: STM32F3.
 *
 * @param dev adc device
 * @param resolution ADC resolution (6, 8, 10 or 12 bit)
 */
void adc_set_resolution(const adc_dev *dev, adc_resolution res);

/**
 * @brief Enable the voltage regulatr of an ADC peripheral
 *
 * Availability: STM32F3.
 *
 * @param dev adc device
 */
void adc_regulator_enable(const adc_dev *dev);

/**
 * @brief Disable the voltage regulatr of an ADC peripheral
 *
 * Availability: STM32F3.
 *
 * @param dev adc device
 */
void adc_regulator_disable(const adc_dev *dev);


/* Pull in the series header
 *
 * IMPORTANT: The series header must define the following:
 *
 * - enum adc_extsel_event (and typedef to adc_extsel_event): One per
 *   external event used to trigger start of conversion of a regular
 *   group. If two different series support the same event as a
 *   trigger, they must use the same token for the enumerator for that
 *   event. (The value of the enumerator is of course allowed to be
 *   different).
 *
 * - enum adc_smp_rate (and typedef to adc_smp_rate): One per
 *   available sampling time.  These must be in the form ADC_SMPR_X_Y
 *   for X.Y cycles (e.g. ADC_SMPR_1_5 means 1.5 cycles), or
 *   ADC_SMPR_X for X cycles (e.g. ADC_SMPR_3 means 3 cycles).
 *
 * - enum adc_prescaler (and typedef): One per available prescaler,
 *   suitable for adc_set_prescaler. Series which have the same
 *   prescaler dividers (e.g. STM32F1 and STM32F2 both divide PCLK2 by
 *   2, 4, 6, or 8) must provide the same tokens as enumerators, for
 *   portability.
 */

/*
 * Routines
 */

extern void adc_init(const adc_dev *dev);

/**
 * @brief Set external event select for regular group
 * @param dev ADC device
 * @param event Event used to trigger the start of conversion.
 * @see adc_extsel_event
 */
extern void adc_set_extsel(const adc_dev *dev, adc_extsel_event event);

/**
 * @brief Set the sample rate for all channels on an ADC device.
 *
 * Don't call this during conversion.
 *
 * @param dev adc device
 * @param smp_rate sample rate to set
 * @see adc_smp_rate
 */
extern void adc_set_sample_rate(const adc_dev *dev, adc_smp_rate smp_rate);

/**
 * @brief Enable scan mode for an ADC.
 *
 * In scan mode, the ADC converts all channels in a regular or
 * injected sequence. After each conversion is done, the ADC
 * automatically starts converting the next channel in the sequence.
 *
 * Scan mode is disabled by default.
 *
 * @see adc_disable_scan()
 */
extern void adc_enable_scan(const adc_dev *dev);

/**
 * @brief Disable scan mode for an ADC.
 *
 * This is the default setting.
 *
 * @see adc_enable_scan()
 */
extern void adc_disable_scan(const adc_dev *dev);

/**
 * @brief Enable continuous mode for an ADC.
 *
 * In this mode, the ADC will automatically perform conversions
 * continuously until taken out of this mode or disabled.
 *
 * Continuous mode is disabled by default.
 *
 * @see adc_disable_continuous()
 */
extern void adc_enable_continuous(const adc_dev *dev);

/**
 * @brief Disable continuous mode for an ADC.
 *
 * This is the default setting.
 *
 * @see adc_enable_continuous()
 */
extern void adc_disable_continuous(const adc_dev *dev);

/**
 * @brief Set the sequence of channels to convert.
 *
 * This sets the (regular) sequence of up to 16 channels to convert.
 *
 * @param dev ADC device
 * @param channels ADC channels to convert; these can repeat and may
 *                 be in any order.
 * @param len Length of `channels', from 1 to 16.
 * @see adc_start_conv()
 */
extern void adc_set_conv_seq(const adc_dev *dev, const uint8 *channels, uint8 len);

/**
 * @brief Attach an interrupt handler and enable its interrupts.
 *
 * This function sets `handler' as the function to be called when any
 * ADC interrupts for `dev' occur. At most one ADC interrupt handler
 * may be attached at any time. Subsequent calls to this function will
 * overwrite any previously attached handler.
 *
 * When `handler' is called, its argument will point to a struct
 * adc_callback_data. The .irq_flags field in this struct will be a
 * logical OR of adc_interrupt_id values encoding the reason(s) for
 * the call. Its .arg field will be the `arg' argument to this
 * function.
 *
 * The interrupt bits set in the adc_callback_data's .irq_flags will
 * always be a subset of those set in the `interrupt_flags' argument
 * to this function. That is, interrupts not given here in the
 * `interrupt_flags' argument will never cause `handler' to be
 * called. This has the effect that any enabled ADC interrupts not
 * specified in `interrupt_flags' will be ignored.
 *
 * This function additionally enables the ADC interrupts specified by
 * `interrupt_flags'.
 *
 * @param dev ADC device whose interrupts to attach to.
 * @param interrupt_flags Logical or of adc_interrupt_id values
 *                        specifying interrupts to enable.
 * @param handler Interrupt handler to call when any interrupt in
 *                interrupt_flags occurs.
 * @param arg Value to store in .arg field of handler's callback data.
 * @see enum adc_interrupt_id
 * @see struct adc_callback_data
 */
extern void adc_attach_interrupt(const adc_dev *dev, uint32 interrupt_flags,
																void (*handler)(adc_callback_data*), void *arg);

/**
 * @brief Disable ADC interrupts and detach interrupt handlers.
 *
 * This function disables all interrupts for `dev', and unsets any
 * handler attached with adc_attach_interrupt().
 *
 * @param dev ADC device whose handler to detach.
 */
extern void adc_detach_interrupt(const adc_dev *dev);

/**
 * @brief Enable ADC interrupts
 * @param dev ADC device
 * @param interrupt_flags Logical or of adc_interrupt_id values to enable.
 * @see adc_disable_interrupt()
 * @see adc_attach_interrupt()
 */
extern void adc_enable_interrupts(const adc_dev *dev, uint32 interrupt_flags);

/**
 * @brief Disable ADC interrupts.
 * @param dev ADC device
 * @param interrupt_flags Logical or of adc_interrupt_id values to disable.
 * @brief adc_enable_interrupt()
 */
extern void adc_disable_interrupts(const adc_dev *dev, uint32 interrupt_flags);

/**
 * @brief Perform a single synchronous software triggered conversion on a
 *        channel.
 * @param dev ADC device to use for reading.
 * @param channel channel to convert
 * @return conversion result
 */
extern uint16 adc_read(const adc_dev *dev, uint8 channel);

/**
 * @brief Set the ADC prescaler.
 *
 * This determines the ADC clock for all devices.
 */
extern void adc_set_prescaler(adc_prescaler pre);

/**
 * @brief Call a function on all ADC devices.
 * @param fn Function to call on each ADC device.
 */
extern void adc_foreach(void (*fn)(const adc_dev*));

struct gpio_dev;
/**
 * @brief Configure a GPIO pin for ADC conversion.
 * @param dev ADC device to use for conversion (currently ignored on
 *            all targets).
 * @param gdev GPIO device to configure.
 * @param bit Bit on gdev to configure for ADC conversion.
 */
extern void adc_config_gpio(const adc_dev *dev, uint8 pin);

/**
 * @brief Enable an ADC and configure it for single conversion mode.
 *
 * This function performs any initialization necessary to allow the
 * ADC device to perform a single synchronous regular software
 * triggered conversion, using adc_read().
 *
 * @param dev Device to enable.
 * @see adc_read()
 */
extern void adc_enable_single_swstart(const adc_dev* dev);

/**
 * @brief Set the regular channel sequence length.
 *
 * Defines the total number of conversions in the regular channel
 * conversion sequence.
 *
 * @param dev ADC device.
 * @param length Regular channel sequence length, from 1 to 16.
 */
extern void adc_set_reg_seqlen(const adc_dev *dev, uint8 length);

/**
 * @brief Enable an adc peripheral
 * @param dev ADC device to enable
 */
extern void adc_enable(const adc_dev *dev);

/**
 * @brief Disable an ADC peripheral
 * @param dev ADC device to disable
 */
extern void adc_disable(const adc_dev *dev);

/**
 * @brief Disable all ADC peripherals.
 */
static inline void adc_disable_all(void) {
    adc_foreach(adc_disable);
}

/*
 * private
 */
extern void _adc_enable_dev_irq(const adc_dev *dev);

#ifdef __cplusplus
}
#endif




#endif
