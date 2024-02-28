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
 * @author Marti Bolivar <mbolivar@leaflabs.com>
 * @brief Wirish SPI implementation.
 */

#include "SPI.h"


#include <libmaple/timer.h>
#include <libmaple/util.h>
#include <libmaple/rcc.h>

#include "wirish.h"
#include "boards.h"

/** Time in ms for DMA receive timeout */
#define DMA_TIMEOUT 100

#if 0 // debug
  #include <stdio.h>
  #include <usb_serial.h>
  static char dbg_s[500];
  #define PRINTF(...) { sprintf(dbg_s, __VA_ARGS__); Serial.print(dbg_s); }
// extern void PRINTF(const char *, ...);
#else
#define PRINTF(...)
#endif


//#define SPI1_ALTERNATE_CONFIG 1 // use alternate SPI1 on SPI3 pins

static const spi_pins_t board_spi_pins[BOARD_NR_SPI] __FLASH__ =
{
    {BOARD_SPI1_NSS_PIN,
     BOARD_SPI1_SCK_PIN,
     BOARD_SPI1_MISO_PIN,
     BOARD_SPI1_MOSI_PIN},
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     BOARD_SPI2_MISO_PIN,
     BOARD_SPI2_MOSI_PIN},
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     BOARD_SPI3_MISO_PIN,
     BOARD_SPI3_MOSI_PIN},
#if (BOARD_NR_SPI>3)
    {BOARD_SPI4_NSS_PIN,
     BOARD_SPI4_SCK_PIN,
     BOARD_SPI4_MISO_PIN,
     BOARD_SPI4_MOSI_PIN},
#endif
#if (BOARD_NR_SPI>4)
    {BOARD_SPI5_NSS_PIN,
     BOARD_SPI5_SCK_PIN,
     BOARD_SPI5_MISO_PIN,
     BOARD_SPI5_MOSI_PIN},
#endif
};
#if defined(BOARD_SPI1A_NSS_PIN) && defined(BOARD_SPI2A_NSS_PIN) && defined(BOARD_SPI3A_NSS_PIN)
#define SPI_ALT_PINS 1
static const spi_pins_t board_spi_alt_pins[BOARD_NR_SPI] __FLASH__ =
{
    {BOARD_SPI1A_NSS_PIN,
     BOARD_SPI1A_SCK_PIN,
     BOARD_SPI1A_MISO_PIN,
     BOARD_SPI1A_MOSI_PIN},
    {BOARD_SPI2A_NSS_PIN,
     BOARD_SPI2A_SCK_PIN,
     BOARD_SPI2A_MISO_PIN,
     BOARD_SPI2A_MOSI_PIN},
    {BOARD_SPI3A_NSS_PIN,
     BOARD_SPI3A_SCK_PIN,
     BOARD_SPI3A_MISO_PIN,
     BOARD_SPI3A_MOSI_PIN},
};
#else
#define SPI_ALT_PINS 0
#endif

//-----------------------------------------------------------------------------
//  Auxiliary functions
//-----------------------------------------------------------------------------
static void disable_pwm(uint8_t pin)
{
    const timer_info * i = &timer_map[pin];
    const timer_dev * t_dev = timer_devices[i->index];
    if (i->index && t_dev) {
        timer_set_mode(t_dev, i->channel, TIMER_DISABLED);
    }
}
//-----------------------------------------------------------------------------
static void configure_gpios(SPISettings * crtSetting, bool as_master)
{
	const spi_pins_t *pins = crtSetting->pins;
    disable_pwm(pins->nss);
    disable_pwm(pins->sck);
    disable_pwm(pins->miso);
    disable_pwm(pins->mosi);

    spi_config_gpios(crtSetting->spi_d, as_master, pins);
}
//-----------------------------------------------------------------------------
static const spi_baud_rate baud_rates[8] __FLASH__ =
{
    SPI_BAUD_PCLK_DIV_2,
    SPI_BAUD_PCLK_DIV_4,
    SPI_BAUD_PCLK_DIV_8,
    SPI_BAUD_PCLK_DIV_16,
    SPI_BAUD_PCLK_DIV_32,
    SPI_BAUD_PCLK_DIV_64,
    SPI_BAUD_PCLK_DIV_128,
    SPI_BAUD_PCLK_DIV_256,
};

//-----------------------------------------------------------------------------
//  Note: This assumes you're on a LeafLabs-style board
//  APB2 = CYCLES_PER_MICROSECOND/2 [MHz], APB1 = APB2/2.
//-----------------------------------------------------------------------------
static spi_baud_rate determine_baud_rate(spi_dev_t *dev, uint32_t freq)
{
    uint32_t clock = 0;
    switch (rcc_dev_clk(dev->clk_id))
    {
    case RCC_APB2: clock = STM32_PCLK2; break; // 84 Mhz // SPI1
    case RCC_APB1: clock = STM32_PCLK1; break; // 42 Mhz // SPI2/3
    case RCC_AHB1: break;
    default: break;
    }
    clock /= 2;
    uint8_t i = 0;
    while (i < 7 && freq < clock) {
        clock /= 2;
        i++;
    }
    return baud_rates[i];
}
//-----------------------------------------------------------------------------
static uint16_t ff = 0XFFFF;

SPISettings _settings[BOARD_NR_SPI];

/**
* @brief Waits unti TXE (tx empy) flag set and BSY (busy) flag unset.
*/
#define waitSpiTxEnd(spi_d) { while (spi_is_tx_empty(spi_d) == 0); while (spi_is_busy(spi_d) != 0); }
//-----------------------------------------------------------------------------
//  This function will be called after the stream finished to transfer
//  (read or write) the programmed number of data (bytes or words).
//  Implemented safeguard against setModule().
//-----------------------------------------------------------------------------
void spiEventCallback(uint32 spi_num)
{
    //Serial.write('+');
    SPISettings * crtSetting = &_settings[spi_num];
	spi_mode_t state = crtSetting->state;
	dma_stream_nr_t dmaStream;
	if ( state==SPI_STATE_RECEIVE )
	{
		dmaStream = crtSetting->spiRxDmaStream;
	}
	else if ( state==SPI_STATE_TRANSMIT)
    {
		dmaStream = crtSetting->spiTxDmaStream;
	} else {
		PRINTF("# [%u] : state=%u #", spi_num, state);
        return;
    }

	uint32_t cr = (crtSetting->spiDmaDev)->regs->STREAM[dmaStream].CR;
	// check for half transfer IRQ
	if ( (cr&DMA_CR_HTIE) && (dma_get_isr_bits(crtSetting->spiDmaDev, dmaStream)&DMA_ISR_HTIF) )
    {
		if (crtSetting->trxCallback)
			crtSetting->trxCallback(0); // half transfer
		return;
	}
// transfer complete. stop the DMA if not in circular mode
	if (!(cr&DMA_CR_CIRC))
	{
		while (spi_is_tx_empty(crtSetting->spi_d) == 0); // "5. Wait until TXE=1 ..."
		while (spi_is_busy(crtSetting->spi_d) != 0); // "... and then wait until BSY=0"
		//while (spi_is_rx_nonempty(crtSetting->spi_d));
		spi_tx_dma_disable(crtSetting->spi_d);
		spi_rx_dma_disable(crtSetting->spi_d);
		dma_disable(crtSetting->spiDmaDev, dmaStream);
		crtSetting->state = SPI_STATE_READY;
	}
    if (crtSetting->trxCallback)
        crtSetting->trxCallback(1); // transfer complete
}
//-----------------------------------------------------------------------------
//  DMA call back functions, one per port.
//-----------------------------------------------------------------------------
void _spi1EventCallback(void) { spiEventCallback(0); }

void _spi2EventCallback(void) { spiEventCallback(1); }

void _spi3EventCallback(void) { spiEventCallback(2); }

void _spi4EventCallback(void) { spiEventCallback(3); }

void _spi5EventCallback(void) { spiEventCallback(4); }

voidFuncPtr _spiEventCallbacks[] = {
    _spi1EventCallback,
    _spi2EventCallback,
    _spi3EventCallback,
#if BOARD_NR_SPI >= 4
    _spi4EventCallback,
#endif
#if BOARD_NR_SPI >= 5
    _spi5EventCallback,
#endif
};

//-----------------------------------------------------------------------------
//  Constructor
//-----------------------------------------------------------------------------
SPIClass::SPIClass(uint32_t spi_num)
{
    setModule(spi_num); // SPI channels are called 1 2 and 3 but the array is zero indexed

	//-------------------------------------------------------------------------
	// Init things specific to each SPI device
	// clock divider setup is a bit of hack, and needs to be improved at a later date.
	//-------------------------------------------------------------------------
	//   DMA / Channel / Stream 
	//            Rx             Tx
	// SPI1:  2 / 3 / 0 (2) - 2 / 3 / 3 (5)
	// SPI2:  1 / 0 / 3     - 1 / 0 / 4
	// SPI3:  1 / 0 / 0 (2) - 1 / 0 / 5 (7)
	//-------------------------------------------------------------------------
	if (_settings[0].spi_d==0) // initialize the settings parameters only once
	{
		_settings[0].spi_d = SPI1;
		_settings[0].spiDmaDev = DMA2;
		_settings[0].pins = &board_spi_pins[0];
		_settings[0].clockDivider = determine_baud_rate(_settings[0].spi_d, _settings[0].clock);
		_settings[0].spiDmaChannel = DMA_CH3;
		_settings[0].spiRxDmaStream  = DMA_STREAM0; // alternative: DMA_STREAM2
		_settings[0].spiTxDmaStream  = DMA_STREAM3; // alternative: DMA_STREAM5
		_settings[0].state = SPI_STATE_IDLE;
		_settings[0].dev_index = 0;
		_settings[1].spi_d = SPI2;
		_settings[1].spiDmaDev = DMA1;
		_settings[1].pins = &board_spi_pins[1];
		_settings[1].clockDivider = determine_baud_rate(_settings[1].spi_d, _settings[1].clock);
		_settings[1].spiDmaChannel = DMA_CH0;
		_settings[1].spiRxDmaStream  = DMA_STREAM3; // alternative: -
		_settings[1].spiTxDmaStream  = DMA_STREAM4; // alternative: -
		_settings[1].state = SPI_STATE_IDLE;
		_settings[1].dev_index = 1;
		_settings[2].spi_d = SPI3;
		_settings[2].spiDmaDev = DMA1;
		_settings[2].pins = &board_spi_pins[2];
		_settings[2].clockDivider = determine_baud_rate(_settings[2].spi_d, _settings[2].clock);
		_settings[2].spiDmaChannel = DMA_CH0;
		_settings[2].spiRxDmaStream  = DMA_STREAM0; // alternative: DMA_STREAM2
		_settings[2].spiTxDmaStream  = DMA_STREAM5; // alternative: DMA_STREAM7
		_settings[2].state = SPI_STATE_IDLE;
		_settings[2].dev_index = 2;
#if (BOARD_NR_SPI>3)
		_settings[3].spi_d = SPI4;
		_settings[3].spiDmaDev = DMA2;
		_settings[3].pins = &board_spi_pins[3];
		_settings[3].clockDivider = determine_baud_rate(_settings[3].spi_d, _settings[3].clock);
		_settings[3].spiDmaChannel = DMA_CH4;
		_settings[3].spiRxDmaStream  = DMA_STREAM0; // alternative: DMA_STREAM4
		_settings[3].spiTxDmaStream  = DMA_STREAM1; // alternative: DMA_STREAM4, DMA_CH5
		_settings[3].state = SPI_STATE_IDLE;
		_settings[3].dev_index = 3;
#endif
#if (BOARD_NR_SPI>4)
		_settings[4].spi_d = SPI5;
		_settings[4].spiDmaDev = DMA2;
		_settings[4].pins = &board_spi_pins[4];
		_settings[4].clockDivider = determine_baud_rate(_settings[4].spi_d, _settings[4].clock);
		_settings[4].spiDmaChannel = DMA_CH2;
		_settings[4].spiRxDmaStream  = DMA_STREAM3; // alternative: CH7 & STREAM5
		_settings[4].spiTxDmaStream  = DMA_STREAM4; // alternative: CH5 & STREAM5
		_settings[4].state = SPI_STATE_IDLE;
		_settings[4].dev_index = 4;
#endif
	}
}

//-----------------------------------------------------------------------------
//  Set up/tear down
//-----------------------------------------------------------------------------
void SPIClass::updateSettings(void) {
    uint32_t flags = (
        (_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) |
        _currentSetting->dataSize | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(_currentSetting->spi_d, (spi_baud_rate)_currentSetting->clockDivider,
        (spi_mode)_currentSetting->dataMode, flags);
}

void SPIClass::begin(void)
{
    PRINTF("<b-");
    spi_init(_currentSetting->spi_d);
    configure_gpios(_currentSetting, 1);
    updateSettings();
    // added for DMA callbacks.
    _currentSetting->state = SPI_STATE_READY;
    PRINTF("-b>");
}

void SPIClass::beginSlave(void)
{
    PRINTF("<bS-");
    spi_init(_currentSetting->spi_d);
    configure_gpios(_currentSetting, 0);
    uint32_t flags = (
        (_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) | _currentSetting->dataSize);
    spi_slave_enable(_currentSetting->spi_d, (spi_mode)_currentSetting->dataMode, flags);
    // added for DMA callbacks.
    _currentSetting->state = SPI_STATE_READY;
    PRINTF("-bS>");
}

void SPIClass::end(void)
{
    if (!spi_is_enabled(_currentSetting->spi_d)) {
        return;
    }

    // Follows RM0008's sequence for disabling a SPI in master/slave full duplex mode.
    while (spi_is_rx_nonempty(_currentSetting->spi_d)) {
        // FIXME [0.1.0] remove this once you have an interrupt based driver
        (void)spi_rx_reg(_currentSetting->spi_d);
    }
    waitSpiTxEnd(_currentSetting->spi_d);
    spi_peripheral_disable(_currentSetting->spi_d);
    // added for DMA callbacks.
    // Need to add unsetting the callbacks for the DMA channels.
    _currentSetting->state = SPI_STATE_IDLE;
}

/* Roger Clark added  3 functions */
void SPIClass::setClockDivider(uint32_t clockDivider)
{
    _currentSetting->clockDivider = clockDivider;
    uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_BR);
    _currentSetting->spi_d->regs->CR1 = cr1 | (clockDivider & SPI_CR1_BR);
}

void SPIClass::setBitOrder(BitOrder bitOrder)
{
    _currentSetting->bitOrder = bitOrder;
    uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_LSBFIRST);
    if ( bitOrder==LSBFIRST )   cr1 |= SPI_CR1_LSBFIRST;
    _currentSetting->spi_d->regs->CR1 = cr1;
}

//-----------------------------------------------------------------------------
//  Victor Perez. Added to test changing datasize from 8 to 16 bit modes on the fly.
//  Input parameter should be SPI_CR1_DFF set to 0 or 1 on a 32bit word.
//-----------------------------------------------------------------------------
void SPIClass::setDataSize(uint32_t datasize)
{
    _currentSetting->dataSize = datasize;
    uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_DFF);
    uint8_t en = spi_is_enabled(_currentSetting->spi_d);
    spi_peripheral_disable(_currentSetting->spi_d);
    _currentSetting->spi_d->regs->CR1 = cr1 | (datasize & SPI_CR1_DFF) | en;
}

void SPIClass::setDataMode(uint8_t dataMode)
{
/* Notes.  As far as I can tell, the AVR numbers for dataMode appear to match the numbers required by the STM32

From the AVR doc http://www.atmel.com/images/doc2585.pdf section 2.4

SPI Mode    CPOL    CPHA    Shift SCK-edge  Capture SCK-edge
0           0       0       Falling         Rising
1           0       1       Rising          Falling
2           1       0       Rising          Falling
3           1       1       Falling         Rising


On the STM32 it appears to be

bit 1 - CPOL : Clock polarity
    (This bit should not be changed when communication is ongoing)
    0 : CLK to 0 when idle
    1 : CLK to 1 when idle

bit 0 - CPHA : Clock phase
    (This bit should not be changed when communication is ongoing)
    0 : The first clock transition is the first data capture edge
    1 : The second clock transition is the first data capture edge

If someone finds this is not the case or sees a logic error with this let me know ;-) 
 */
    _currentSetting->dataMode = dataMode;
    uint32_t cr1 = _currentSetting->spi_d->regs->CR1 & ~(SPI_CR1_CPOL|SPI_CR1_CPHA);
    _currentSetting->spi_d->regs->CR1 = cr1 | (dataMode & (SPI_CR1_CPOL|SPI_CR1_CPHA));
}

void SPIClass::beginTransaction(SPISettings settings)
{
    PRINTF("<bT-");
    setBitOrder(settings.bitOrder);
    setDataMode(settings.dataMode);
    setDataSize(settings.dataSize);
    setClockDivider(determine_baud_rate(_currentSetting->spi_d, settings.clock));
    begin();
    PRINTF("-bT>");
}

void SPIClass::beginTransactionSlave(SPISettings settings)
{
    PRINTF("<bTS-");
    setBitOrder(settings.bitOrder);
    setDataMode(settings.dataMode);
    setDataSize(settings.dataSize);
    beginSlave();
    PRINTF("-bTS>");
}

void SPIClass::endTransaction(void)
{
    end();
    // release_gpios(_currentSetting->spi_d, 1);
}

//-----------------------------------------------------------------------------
//  I/O
//-----------------------------------------------------------------------------
uint16_t SPIClass::read(void)
{
    while ( spi_is_rx_nonempty(_currentSetting->spi_d)==0 ) ;
    return (uint16)spi_rx_reg(_currentSetting->spi_d);
}

uint16_t SPIClass::read16(void)
{
    while ( spi_is_rx_nonempty(_currentSetting->spi_d)==0 ) ;
    uint16_t ret = (uint16)spi_rx_reg(_currentSetting->spi_d);
    return (ret << 8) | (uint16)spi_rx_reg(_currentSetting->spi_d);
}

void SPIClass::read(uint8_t *buf, uint16_t len)
{
    PRINTF("<spiRd-%u>\n", len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = 0x00FF;                       // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // avoid interrupts to surely get the previously received data
        regs->DR = 0x00FF;                   // write the next data item to be transmitted into the SPI_DR register.
                                             // This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *buf++ = (uint8)(regs->DR);          // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *buf++ = (uint8)(regs->DR);              // read and store the received byte
}
//-----------------------------------------------------------------------------
//  Added for 16bit data Victor Perez. Roger Clark 
//  Improved speed by just directly writing the single byte to the SPI data reg and wait for completion,
//  by taking the Tx code from transfer(byte)
//  This almost doubles the speed of this function.
//-----------------------------------------------------------------------------
void SPIClass::write(const uint16_t data)
{   // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
    PRINTF("<spiWr-%04X>\n", data);
    ASSERT((uint32_t)_currentSetting->spi_d < 0x50000000);
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(_currentSetting->spi_d, data);
}
//-----------------------------------------------------------------------------
//  Added by stevestrong: write two consecutive bytes in 8 bit mode (DFF=0)
//-----------------------------------------------------------------------------
void SPIClass::write16(const uint16_t data)
{
    PRINTF("<spiWr16-%04X>\n", data);
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(_currentSetting->spi_d, data>>8); // write high byte
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(_currentSetting->spi_d, data); // write low byte
    // waitSpiTxEnd(_currentSetting->spi_d);
}
//-----------------------------------------------------------------------------
//  Added by stevstrong: Repeatedly send same data by the specified number of times
//-----------------------------------------------------------------------------
void SPIClass::write(const uint16_t data, uint16_t n)
{
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    while ( (n--)>0 ) {
        while ( (regs->SR & SPI_SR_TXE)==0 ) ; // wait till Tx empty
        regs->DR = data; // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
    }
    // while ( (regs->SR & SPI_SR_BSY) != 0); // wait until BSY=0 before returning
}

void SPIClass::write(const void *data, uint16_t length)
{
    PRINTF("<spiWrBuf-%u>\n", length);
    spi_dev_t * spi_d = _currentSetting->spi_d;
    spi_tx(spi_d, data, length); // data can be array of bytes or words
    // waitSpiTxEnd(spi_d); // "5. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI."
}
#if 0 // no need to overload, uint16_t is good for uint8_t as well
uint8_t SPIClass::transfer(uint8_t byte) const
{
    spi_dev_t * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d); // read any previous data
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(spi_d, byte); // Write the data item to be transmitted into the SPI_DR register
    waitSpiTxEnd(spi_d);
    return (uint8)spi_rx_reg(spi_d); // "... and read the last received data."
}
#endif
uint16_t SPIClass::transfer(uint16_t data) const
{
    spi_dev_t * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d); // read any previous data
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(spi_d, data); // Write the data item to be transmitted into the SPI_DR register
    waitSpiTxEnd(spi_d);
    return (uint16_t)spi_rx_reg(spi_d); // "... and read the last received data."
}
//-----------------------------------------------------------------------------
//  Modified by stevestrong: write & read two consecutive bytes in 8 bit mode (DFF=0)
//  This is more effective than two distinct byte transfers
//-----------------------------------------------------------------------------
uint16_t SPIClass::transfer16(const uint16_t data) const
{
    PRINTF("<trx16-%04X...", data);
    // Modified by stevestrong: write & read two consecutive bytes in 8 bit mode (DFF=0)
    // This is more effective than two distinct byte transfers
    spi_dev_t * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d);                   // read any previous data
    spi_tx_reg(spi_d, data>>8);          // write high byte
    waitSpiTxEnd(spi_d);                 // wait until TXE=1 and then wait until BSY=0
    uint16_t ret = spi_rx_reg(spi_d)<<8; // read and shift high byte
    spi_tx_reg(spi_d, data);             // write low byte
    waitSpiTxEnd(spi_d);                 // wait until TXE=1 and then wait until BSY=0
    ret += spi_rx_reg(spi_d);            // read low byte
    PRINTF("trx16-%04X>\n", ret);
    return ret;
}

void SPIClass::transfer(const uint8_t * tx_buf, uint8_t * rx_buf, uint16_t len)
{
    PRINTF("<t8bbl-%u>\n", len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = *tx_buf++;                    // write the first byte
    // main loop
    while ( (--len) ) {
        while ( !(regs->SR & SPI_SR_TXE) );  // wait for TXE flag
        noInterrupts();                      // avoid interrupts to surely get the previously received data
        regs->DR = *tx_buf++;                // write the next data item to be transmitted into the SPI_DR register.
                                             // This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = (uint8)(regs->DR);       // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = (uint8)(regs->DR);           // read and store the received byte
}

void SPIClass::transfer(const uint8_t tx_data, uint8_t * rx_buf, uint16_t len)
{
    PRINTF("<t8dbl-%u-%u>\n", tx_data, len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = tx_data;                      // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // avoid interrupts to surely get the previously received data
        regs->DR = tx_data;                  // write the next data item to be transmitted into the SPI_DR register.
                                             // This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = (uint8)(regs->DR);       // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = (uint8)(regs->DR);           // read and store the received byte
}

void SPIClass::transfer(const uint16_t * tx_buf, uint16_t * rx_buf, uint16_t len)
{
    PRINTF("<t16dbl-%u>\n", len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = *tx_buf++;                    // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // avoid interrupts to surely get the previously received data
        regs->DR = *tx_buf++;                // write the next data item to be transmitted into the SPI_DR register.
                                             // This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = regs->DR;                // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = regs->DR;                    // read and store the received byte
}

void SPIClass::transfer(const uint16_t tx_data, uint16_t * rx_buf, uint16_t len)
{
    PRINTF("<t16dbl-%u-%u>\n", tx_data, len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = tx_data;                      // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // avoid interrupts to surely get the previously received data
        regs->DR = tx_data;                  // write the next data item to be transmitted into the SPI_DR register.
                                             // This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = regs->DR;                // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = regs->DR;                    // read and store the received byte
}

//-----------------------------------------------------------------------------
//  Roger Clark and Victor Perez, 2015
//  Performs a DMA SPI transfer with at least a receive buffer.
//  If a TX buffer is not provided, FF is sent over and over for the lenght of the transfer. 
//  On exit TX buffer is not modified, and RX buffer cotains the received data.
//  Still in progress.
//-----------------------------------------------------------------------------
//  Changed by stevestrong:
//  - DMA IRQ event will always be triggered
//  - inserted yield() on waiting for DMA end
//  - added half transfer monitoring
//-----------------------------------------------------------------------------
//  Wait for DMA to finish its job
//-----------------------------------------------------------------------------
void SPIClass::dmaWaitCompletion(void)
{
    PRINTF("<dWC-");
    if (spi_is_master(_currentSetting->spi_d) && _currentSetting->state != SPI_STATE_READY)
    {
        uint32_t m = millis();
        while ( _currentSetting->state != SPI_STATE_READY )
        {
            yield(); // do something in main loop

            if ((millis()-m)>DMA_TIMEOUT)
            {
                PRINTF("DMA timeout: state: %s", (_currentSetting->state==SPI_STATE_RECEIVE)? "Rx":"Tx");
                PRINTF("\nDMA[Tx] CR: %04X", (_currentSetting->spiDmaDev)->regs->STREAM[_currentSetting->spiTxDmaStream].CR);
                PRINTF(", NDTR: %04X", (_currentSetting->spiDmaDev)->regs->STREAM[_currentSetting->spiTxDmaStream].NDTR);
                PRINTF(", LISR: %04X", (_currentSetting->spiDmaDev)->regs->LISR);
                PRINTF("\nDMA[Rx] CR: %04X", (_currentSetting->spiDmaDev)->regs->STREAM[_currentSetting->spiRxDmaStream].CR);
                PRINTF(", NDTR: %04X", (_currentSetting->spiDmaDev)->regs->STREAM[_currentSetting->spiRxDmaStream].NDTR);
                PRINTF(", LISR: %04X", (_currentSetting->spiDmaDev)->regs->LISR);
                PRINTF("\n");
                // disable DMA
                waitSpiTxEnd(_currentSetting->spi_d); // Wait until TXE=1 and then wait until BSY=0"
                spi_tx_dma_disable(_currentSetting->spi_d);
                spi_rx_dma_disable(_currentSetting->spi_d);
                dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream);
                dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream);
                //while(1);
                _currentSetting->state = SPI_STATE_READY;
                break;
            }
        }
    }
    PRINTF("-dWC>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransferSet(const void *txBuf, void *rxBuf, uint16_t flags)
{
    PRINTF("<dTS-");
    dmaWaitCompletion();
    dma_init(_currentSetting->spiDmaDev);
    if (spi_is_master(_currentSetting->spi_d) && !(flags&DMA_CIRC_MODE))
        flags |= DMA_TRNS_CMPLT; // disable DMA after transfer
    dma_xfer_size dma_bit_size = (_currentSetting->dataSize==SPI_DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
    // RX
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream,
                       _currentSetting->spiDmaChannel, dma_bit_size,
                       &_currentSetting->spi_d->regs->DR, // peripheral address
                       rxBuf,                             // memory bank 0 address
                       NULL,                              // memory bank 1 address
                       (flags | (DMA_MINC_MODE|DMA_FROM_PER|DMA_PRIO_VERY_HIGH)));
    if (flags&(DMA_TRNS_CMPLT|DMA_TRNS_HALF))
        dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream,
            _spiEventCallbacks[_currentSetting->dev_index]);
    // TX
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream,
                       _currentSetting->spiDmaChannel, dma_bit_size,
                       &_currentSetting->spi_d->regs->DR, // peripheral address
                       (void*)txBuf,                      // memory bank 0 address
                       NULL,                              // memory bank 1 address
                       (flags | DMA_FROM_MEM|DMA_PRIO_HIGH));
    PRINTF("-dTS>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransferRepeat(void * txBuf, void * rxBuf)
{
    PRINTF("<dTR-");
    dmaWaitCompletion();
    _currentSetting->state = SPI_STATE_RECEIVE;
    // RX
    if (rxBuf)
        dma_set_mem_addr(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream, rxBuf);
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaStream);
    // TX
    if (txBuf)
        dma_set_mem_addr(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream, txBuf);
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream);
    // software enable sequence, see AN4031, chapter 4.3
    spi_rx_reg(_currentSetting->spi_d); // pre-empty Rx pipe
    spi_rx_dma_enable(_currentSetting->spi_d);
    spi_tx_dma_enable(_currentSetting->spi_d);
    if (!_currentSetting->dmaTrxAsync) // wait for trx completion if async flag is not set
        dmaWaitCompletion();
    PRINTF("-dTR>");
}

//-----------------------------------------------------------------------------
//  Roger Clark and Victor Perez, 2015, stevestrong 2018
//  Performs a DMA SPI transfer with at least a receive buffer.
//  If a TX buffer is not provided, FF is sent over and over for the length of the transfer. 
//  On exit TX buffer is not modified, and RX buffer contains the received data.
//  Still in progress.
//-----------------------------------------------------------------------------
void SPIClass::dmaTransfer(const void *txBuf, void *rxBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dT-");
    dmaWaitCompletion();
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaTransferSet(txBuf, rxBuf, (flags | DMA_MINC_MODE));
    dmaTransferRepeat();
    PRINTF("-dT>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransfer(const uint16_t tx_data, void *rxBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dT-");
    dmaWaitCompletion();
    ff = tx_data;
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaTransferSet(&ff, rxBuf, flags);
    dmaTransferRepeat();
    PRINTF("-dT>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransferInit(const void *txBuf, void *rxBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dTI-");
    dmaWaitCompletion();
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaTransferSet(txBuf, rxBuf, (flags | DMA_MINC_MODE));
    PRINTF("-dTI>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransferInit(const uint16_t tx_data, void *rxBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dTI-");
    dmaWaitCompletion();
    ff = tx_data;
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaTransferSet(&ff, rxBuf, flags);
    PRINTF("-dTI>\n");
}

//-----------------------------------------------------------------------------
//  Roger Clark and Victor Perez, 2015
//  Performs a DMA SPI send using a TX buffer.
//  On exit TX buffer is not modified.
//  Still in progress.
//  2016 - stevstrong - reworked to automatically detect bit size from SPI setting
//-----------------------------------------------------------------------------
void SPIClass::dmaSendSet(const void * txBuf, uint16_t flags)
{
    PRINTF("<dSS-");
    dmaWaitCompletion();
    dma_init(_currentSetting->spiDmaDev);
    dma_xfer_size dma_bit_size = (_currentSetting->dataSize==SPI_DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
    if (!(flags&DMA_CIRC_MODE)) flags |= DMA_TRNS_CMPLT; // disable DMA after transfer
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream,
                       _currentSetting->spiDmaChannel, dma_bit_size,
                       &_currentSetting->spi_d->regs->DR, // peripheral address
                       (void *)txBuf,                     // memory bank 0 address
                       NULL,                              // memory bank 1 address
                       (flags | DMA_FROM_MEM));
    if ((flags&(DMA_TRNS_CMPLT|DMA_TRNS_HALF)))
        dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream,
            _spiEventCallbacks[_currentSetting->dev_index]);
    PRINTF("-dSS>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSendRepeat(void)
{
    PRINTF("<dSR-");
    dmaWaitCompletion();
    _currentSetting->state = SPI_STATE_TRANSMIT;
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaStream);
    spi_tx_dma_enable(_currentSetting->spi_d);
    if (!_currentSetting->dmaTrxAsync) // check async flag
        dmaWaitCompletion();
    PRINTF("-dSR>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSend(const void * txBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dS-");
    dmaWaitCompletion();
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaSendSet(txBuf, flags | DMA_MINC_MODE);
    dmaSendRepeat();
    PRINTF("-dS>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSend(const uint16_t tx_data, uint16_t length, uint16_t flags)
{
    PRINTF("<dS-");
    dmaWaitCompletion();
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    ff = tx_data;
    dmaSendSet(&ff, flags);
    dmaSendRepeat();
    PRINTF("-dS>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSendInit(const void * txBuf, uint16_t length, uint16_t flags)
{
    PRINTF("<dSI-");
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    dmaSendSet(txBuf, flags | DMA_MINC_MODE);
    PRINTF("-dSI>\n");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSendInit(const uint16_t tx_data, uint16_t length, uint16_t flags)
{
    PRINTF("<dSI-");
    _currentSetting->dmaTrxLength = length;
    _currentSetting->dmaTrxAsync = (flags&DMA_ASYNC);
    ff = tx_data;
    dmaSendSet(&ff, flags);
    PRINTF("-dSI>\n");
}


void SPIClass::attachInterrupt(void) {
    // Should be enableInterrupt()
}

void SPIClass::detachInterrupt(void) {
    // Should be disableInterrupt()
}

/*
 * Pin accessors
 */

uint8 SPIClass::misoPin(void) {
    return (_currentSetting->pins)->miso;
}

uint8 SPIClass::mosiPin(void) {
    return (_currentSetting->pins)->mosi;
}

uint8 SPIClass::sckPin(void) {
    return (_currentSetting->pins)->sck;
}

uint8 SPIClass::nssPin(void) {
    return (_currentSetting->pins)->nss;
}

/*
 * Deprecated functions
 */

uint8_t SPIClass::send(uint8_t data) {
    this->write(data);
    return 1;
}

uint8_t SPIClass::send(uint8_t *buf, uint16_t len) {
    this->write(buf, len);
    return len;
}


SPIClass SPI(1);
