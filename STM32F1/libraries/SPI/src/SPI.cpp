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

#define PRINTF(...)
// extern void PRINTF(const char *, ...);

#if CYCLES_PER_MICROSECOND != 72
/* TODO [0.2.0?] something smarter than this */
#warning "Unexpected clock speed; SPI frequency calculation will be incorrect"
#endif

#if (BOARD_NR_SPI >= 3) && !defined(STM32_HIGH_DENSITY)
#error "The SPI library is misconfigured: 3 SPI ports only available on high density STM32 devices"
#endif

static const spi_pins board_spi_pins[BOARD_NR_SPI] __FLASH__ =
{
    {BOARD_SPI1_NSS_PIN,
     BOARD_SPI1_SCK_PIN,
     BOARD_SPI1_MISO_PIN,
     BOARD_SPI1_MOSI_PIN},
#if BOARD_NR_SPI >= 2
    {BOARD_SPI2_NSS_PIN,
     BOARD_SPI2_SCK_PIN,
     BOARD_SPI2_MISO_PIN,
     BOARD_SPI2_MOSI_PIN},
#endif
#if BOARD_NR_SPI >= 3
    {BOARD_SPI3_NSS_PIN,
     BOARD_SPI3_SCK_PIN,
     BOARD_SPI3_MISO_PIN,
     BOARD_SPI3_MOSI_PIN},
#endif
};

//-----------------------------------------------------------------------------
//  Auxiliary functions
//-----------------------------------------------------------------------------
static const spi_pins * dev_to_spi_pins(spi_dev *dev)
{
    switch (dev->clk_id) {
    case RCC_SPI1: return board_spi_pins;
#if BOARD_NR_SPI >= 2
    case RCC_SPI2: return board_spi_pins + 1;
#endif
#if BOARD_NR_SPI >= 3
    case RCC_SPI3: return board_spi_pins + 2;
#endif
    default:       return NULL;
    }
}
//-----------------------------------------------------------------------------
static void disable_pwm(uint8_t pin)
{
    timer_dev * t_dev = PinTimerDevice(pin);
    uint8_t     t_ch  = PinTimerChannel(pin);
    if (t_dev && t_ch) {
        timer_set_mode(t_dev, t_ch, TIMER_DISABLED);
    }
}
//-----------------------------------------------------------------------------
static void configure_gpios(spi_dev *dev, bool as_master)
{
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins) {
        return;
    }

    disable_pwm(pins->nss);
    disable_pwm(pins->sck);
    disable_pwm(pins->miso);
    disable_pwm(pins->mosi);

    spi_config_gpios(as_master, pins);
}
//-----------------------------------------------------------------------------
static void release_gpios(spi_dev *dev, bool as_master)
{
    const spi_pins *pins = dev_to_spi_pins(dev);

    if (!pins)
        return;

    spi_release_gpios(as_master, pins);
}
//-----------------------------------------------------------------------------
static const spi_baud_rate baud_rates[8] __FLASH__ = {
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
//  (CYCLES_PER_MICROSECOND == 72, APB2 at 72MHz, APB1 at 36MHz).
//-----------------------------------------------------------------------------
static spi_baud_rate determine_baud_rate(spi_dev *dev, uint32_t freq)
{
    uint32_t clock = 0;
	rcc_clk_domain clk_domain = rcc_dev_clk(dev->clk_id);
    if (clk_domain==RCC_APB2)      clock = STM32_PCLK2; // 72 Mhz
    else if (clk_domain==RCC_APB1) clock = STM32_PCLK1; // 36 Mhz
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
void spiEventCallback(uint32_t spi_num)
{
    SPISettings * crtSetting = &_settings[spi_num];
    dma_channel dmaChannel = (crtSetting->state==SPI_STATE_TRANSMIT) ? crtSetting->spiTxDmaChannel :
                             ( (crtSetting->state==SPI_STATE_RECEIVE) ? crtSetting->spiRxDmaChannel : (dma_channel)-1);

    if ( dmaChannel==(dma_channel)-1 )
    {
        PRINTF("SPI event: wrong state = %u",crtSetting->state);
        return;
    }

	uint32_t ccr = dma_channel_regs(crtSetting->spiDmaDev, dmaChannel)->CCR;
	// check for half transfer IRQ
	if ( (ccr&DMA_CCR_HTIE) && (dma_get_isr_bits(crtSetting->spiDmaDev, dmaChannel)&DMA_ISR_HTIF) )
	{
		if (crtSetting->trxCallback)
			crtSetting->trxCallback(0); // half transfer
		return;
	}
	// transfer complete. stop the DMA if not in circular mode
	if (!(ccr&DMA_CCR_CIRC))
	{
		waitSpiTxEnd(crtSetting->spi_d); // Wait until TXE=1 and then wait until BSY=0"
		//while (spi_is_rx_nonempty(crtSetting->spi_d));
		spi_tx_dma_disable(crtSetting->spi_d);
		spi_rx_dma_disable(crtSetting->spi_d);
		//dma_disable(crtSetting->spiDmaDev, crtSetting->spiRxDmaChannel);
		//dma_disable(crtSetting->spiDmaDev, crtSetting->spiTxDmaChannel);
		crtSetting->state = SPI_STATE_READY;
	}
    if (crtSetting->trxCallback)
        crtSetting->trxCallback(1); // transfer complete
}
//-----------------------------------------------------------------------------
//  DMA call back functions, one per port.
//-----------------------------------------------------------------------------
void _spi1EventCallback(void) { spiEventCallback(0); }

#if BOARD_NR_SPI >= 2
void _spi2EventCallback(void) { spiEventCallback(1); }
#endif

#if BOARD_NR_SPI >= 3
void _spi3EventCallback(void) { spiEventCallback(2); }
#endif

voidFuncPtr _spiEventCallbacks[] = {
    _spi1EventCallback,
#if BOARD_NR_SPI >= 2
    _spi2EventCallback,
#endif
#if BOARD_NR_SPI >= 3
    _spi3EventCallback,
#endif
};
//-----------------------------------------------------------------------------
//  Constructor
//-----------------------------------------------------------------------------
SPIClass::SPIClass(uint32_t spi_num)
{
    _currentSetting = &_settings[spi_num-1]; // SPI channels are called 1 2 and 3 but the array is zero indexed

	//-------------------------------------------------------------------------
	// Init things specific to each SPI device
	// clock divider setup is a bit of hack, and needs to be improved at a later date.
	//-------------------------------------------------------------------------
	if (_settings[0].spi_d==0) // initialize the settings parameters only once
	{
		_settings[0].spi_d = SPI1;
		_settings[0].spiDmaDev = DMA1;
		_settings[0].clockDivider = determine_baud_rate(_settings[0].spi_d, _settings[0].clock);
		_settings[0].spiRxDmaChannel = DMA_CH2;
		_settings[0].spiTxDmaChannel = DMA_CH3;
		_settings[0].state = SPI_STATE_IDLE;
		_settings[0].dev_index = 0;
#if BOARD_NR_SPI >= 2
		_settings[1].spi_d = SPI2;
		_settings[1].spiDmaDev = DMA1;
		_settings[1].clockDivider = determine_baud_rate(_settings[1].spi_d, _settings[1].clock);
		_settings[1].spiRxDmaChannel = DMA_CH4;
		_settings[1].spiTxDmaChannel = DMA_CH5;
		_settings[1].state = SPI_STATE_IDLE;
		_settings[1].dev_index = 1;
#endif
#if BOARD_NR_SPI >= 3
		_settings[2].spi_d = SPI3;
		_settings[2].spiDmaDev = DMA2;
		_settings[2].clockDivider = determine_baud_rate(_settings[2].spi_d, _settings[2].clock);
		_settings[2].spiRxDmaChannel = DMA_CH1;
		_settings[2].spiTxDmaChannel = DMA_CH2;
		_settings[2].state = SPI_STATE_IDLE;
		_settings[2].dev_index = 2;
#endif
	}
}

//-----------------------------------------------------------------------------
//  Set up/tear down
//-----------------------------------------------------------------------------
void SPIClass::updateSettings(void) {
    uint32_t flags = ((_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) | _currentSetting->dataSize | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(_currentSetting->spi_d, (spi_baud_rate)_currentSetting->clockDivider, (spi_mode)_currentSetting->dataMode, flags);
}

void SPIClass::begin(void)
{
    PRINTF("<b-");
    spi_init(_currentSetting->spi_d);
    configure_gpios(_currentSetting->spi_d, 1);
    updateSettings();
    // added for DMA callbacks.
    _currentSetting->state = SPI_STATE_READY;
    PRINTF("-b>");
}

void SPIClass::beginSlave(void)
{
    PRINTF("<bS-");
    spi_init(_currentSetting->spi_d);
    configure_gpios(_currentSetting->spi_d, 0);
    uint32_t flags = ((_currentSetting->bitOrder == MSBFIRST ? SPI_FRAME_MSB : SPI_FRAME_LSB) | _currentSetting->dataSize);
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

    // Follows RM0008's sequence for disabling a SPI in master/slave
    // full duplex mode.
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
    release_gpios(_currentSetting->spi_d, 1);
}

//-----------------------------------------------------------------------------
//  I/O
//-----------------------------------------------------------------------------
uint16_t SPIClass::read(void)
{
    while ( spi_is_rx_nonempty(_currentSetting->spi_d)==0 ) ;
    return (uint16)spi_rx_reg(_currentSetting->spi_d);
}

void SPIClass::read(uint8_t *buf, uint32_t len)
{
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = 0x00FF;                       // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // go atomic level - avoid interrupts to surely get the previously received data
        regs->DR = 0x00FF;                   // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
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
{
    spi_tx_reg(_currentSetting->spi_d, data); // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
    waitSpiTxEnd(_currentSetting->spi_d);
}
//-----------------------------------------------------------------------------
//  Added by stevestrong: write two consecutive bytes in 8 bit mode (DFF=0)
//-----------------------------------------------------------------------------
void SPIClass::write2(const uint16_t data)
{
    spi_tx_reg(_currentSetting->spi_d, data>>8); // write high byte
    while (spi_is_tx_empty(_currentSetting->spi_d) == 0); // Wait until TXE=1
    spi_tx_reg(_currentSetting->spi_d, data); // write low byte
    waitSpiTxEnd(_currentSetting->spi_d);
}
//-----------------------------------------------------------------------------
//  Added by stevstrong: Repeatedly send same data by the specified number of times
//-----------------------------------------------------------------------------
void SPIClass::write(const uint16_t data, uint32_t n)
{
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    while ( (n--)>0 ) {
        regs->DR = data; // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
        while ( (regs->SR & SPI_SR_TXE)==0 ) ; // wait till Tx empty
    }
    while ( (regs->SR & SPI_SR_BSY) != 0); // wait until BSY=0 before returning 
}

void SPIClass::write(const void *data, uint32_t length)
{
    spi_dev * spi_d = _currentSetting->spi_d;
    spi_tx(spi_d, data, length); // data can be array of bytes or words
    waitSpiTxEnd(spi_d); // "5. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI."
}

uint8_t SPIClass::transfer(uint8_t byte) const
{
    spi_dev * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d); // read any previous data
    spi_tx_reg(spi_d, byte); // Write the data item to be transmitted into the SPI_DR register
    waitSpiTxEnd(spi_d);
    return (uint8)spi_rx_reg(spi_d); // "... and read the last received data."
}
uint16_t SPIClass::transfer(uint16_t data) const
{
    spi_dev * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d); // read any previous data
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
    // Modified by stevestrong: write & read two consecutive bytes in 8 bit mode (DFF=0)
    // This is more effective than two distinct byte transfers
    spi_dev * spi_d = _currentSetting->spi_d;
    spi_rx_reg(spi_d);                   // read any previous data
    spi_tx_reg(spi_d, data>>8);          // write high byte
    waitSpiTxEnd(spi_d);                 // wait until TXE=1 and then wait until BSY=0
    uint16_t ret = spi_rx_reg(spi_d)<<8; // read and shift high byte
    spi_tx_reg(spi_d, data);             // write low byte
    waitSpiTxEnd(spi_d);                 // wait until TXE=1 and then wait until BSY=0
    ret += spi_rx_reg(spi_d);            // read low byte
    return ret;
}

void SPIClass::transfer(const uint8_t * tx_buf, uint8_t * rx_buf, uint32_t len)
{
    PRINTF("<trx-%u",len);
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = *tx_buf++;                    // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) ) yield();   // wait for TXE flag
        noInterrupts();                      // go atomic level - avoid interrupts to surely get the previously received data
        regs->DR = *tx_buf++;                // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ) yield(); // wait till data is available in the DR register
        *rx_buf++ = (uint8)(regs->DR);       // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = (uint8)(regs->DR);           // read and store the received byte
    PRINTF("-trx>");
}

void SPIClass::transfer(const uint8_t tx_data, uint8_t * rx_buf, uint32_t len)
{
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = tx_data;                      // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // go atomic level - avoid interrupts to surely get the previously received data
        regs->DR = tx_data;                  // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = (uint8)(regs->DR);       // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = (uint8)(regs->DR);           // read and store the received byte
}

void SPIClass::transfer(const uint16_t * tx_buf, uint16_t * rx_buf, uint32_t len)
{
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = *tx_buf++;                    // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // go atomic level - avoid interrupts to surely get the previously received data
        regs->DR = *tx_buf++;                // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
        while ( !(regs->SR & SPI_SR_RXNE) ); // wait till data is available in the DR register
        *rx_buf++ = regs->DR;                // read and store the received byte. This clears the RXNE flag.
        interrupts();                        // let systick do its job
    }
    // read remaining last byte
    while ( !(regs->SR & SPI_SR_RXNE) );     // wait till data is available in the Rx register
    *rx_buf++ = regs->DR;                    // read and store the received byte
}

void SPIClass::transfer(const uint16_t tx_data, uint16_t * rx_buf, uint32_t len)
{
    if ( len == 0 ) return;
    spi_rx_reg(_currentSetting->spi_d);      // clear the RX buffer in case a byte is waiting on it.
    spi_reg_map * regs = _currentSetting->spi_d->regs;
    // start sequence: write byte 0
    regs->DR = tx_data;                      // write the first byte
    // main loop
    while ( (--len) ) {
        while( !(regs->SR & SPI_SR_TXE) );   // wait for TXE flag
        noInterrupts();                      // go atomic level - avoid interrupts to surely get the previously received data
        regs->DR = tx_data;                  // write the next data item to be transmitted into the SPI_DR register. This clears the TXE flag.
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
    if (_currentSetting->state != SPI_STATE_READY)
    {
        uint32_t m = millis();
        while ( _currentSetting->state != SPI_STATE_READY )
        {
            yield(); // do something in main loop

            if ((millis()-m)>DMA_TIMEOUT)
            {
                PRINTF("DMA1 timeout, state: %u", _currentSetting->state);
                //Serial.print("DMA timeout: "); Serial.println(_currentSetting->dmaTimeout);
                PRINTF(", CCR2: %04x", DMA1->regs->CCR2);
                PRINTF(", CCR3: %04x", DMA1->regs->CCR3);
                PRINTF(", CNDTR2: %u", DMA1->regs->CNDTR2);
                PRINTF(", CNDTR3: %u", DMA1->regs->CNDTR3);
                PRINTF(", CCR4: %04x", DMA1->regs->CCR4);
                PRINTF(", CCR5: %04x", DMA1->regs->CCR5);
                PRINTF(", CNDTR4: %u", DMA1->regs->CNDTR4);
                PRINTF(", CNDTR5: %u", DMA1->regs->CNDTR5);
                PRINTF("\n");
                // disable DMA
                waitSpiTxEnd(_currentSetting->spi_d); // Wait until TXE=1 and then wait until BSY=0"
                spi_tx_dma_disable(_currentSetting->spi_d);
                spi_rx_dma_disable(_currentSetting->spi_d);
                dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
                dma_disable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
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
    if (!(flags&DMA_CIRC_MODE)) flags |= DMA_TRNS_CMPLT; // disable DMA after transfer
    dma_xfer_size dma_bit_size = (_currentSetting->dataSize==SPI_DATA_SIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
    // RX
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel,
                       &_currentSetting->spi_d->regs->DR, dma_bit_size,
                       rxBuf, dma_bit_size,
                       (flags | DMA_MINC_MODE));
    dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, DMA_PRIORITY_VERY_HIGH);
    if (flags&(DMA_TRNS_CMPLT|DMA_HALF_TRNS))
        dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, _spiEventCallbacks[_currentSetting->dev_index]);
    // TX
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel,
                       &_currentSetting->spi_d->regs->DR, dma_bit_size,
                       (void*)txBuf, dma_bit_size,
                       (flags | DMA_FROM_MEM));
    dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, DMA_PRIORITY_HIGH);
    PRINTF("-dTS>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaTransferRepeat()
{
    PRINTF("<dTR-");
    dmaWaitCompletion();
    _currentSetting->state = SPI_STATE_RECEIVE;
    // RX
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiRxDmaChannel);
    // TX
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    // enable SPI DMA
    spi_rx_reg(_currentSetting->spi_d); // pre-empty Rx pipe
    spi_rx_dma_enable(_currentSetting->spi_d);
    spi_tx_dma_enable(_currentSetting->spi_d);
    // if (!_currentSetting->dmaTrxAsync)
    //     dmaWaitCompletion();
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
    dma_setup_transfer(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel,
                       &_currentSetting->spi_d->regs->DR, dma_bit_size,
                       (void *)txBuf, dma_bit_size,
                       (flags | DMA_FROM_MEM));
    dma_set_priority(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, DMA_PRIORITY_LOW);
    if ((flags&(DMA_TRNS_CMPLT|DMA_HALF_TRNS)))
        dma_attach_interrupt(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, _spiEventCallbacks[_currentSetting->dev_index]);
    PRINTF("-dSS>");
}
//-----------------------------------------------------------------------------
void SPIClass::dmaSendRepeat(void)
{
    PRINTF("<dSR-");
    dmaWaitCompletion();
    _currentSetting->state = SPI_STATE_TRANSMIT;
    dma_set_num_transfers(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel, _currentSetting->dmaTrxLength);
    dma_clear_isr_bits(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
    dma_enable(_currentSetting->spiDmaDev, _currentSetting->spiTxDmaChannel);
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

uint8_t SPIClass::misoPin(void) {
    return dev_to_spi_pins(_currentSetting->spi_d)->miso;
}

uint8_t SPIClass::mosiPin(void) {
    return dev_to_spi_pins(_currentSetting->spi_d)->mosi;
}

uint8_t SPIClass::sckPin(void) {
    return dev_to_spi_pins(_currentSetting->spi_d)->sck;
}

uint8_t SPIClass::nssPin(void) {
    return dev_to_spi_pins(_currentSetting->spi_d)->nss;
}

/*
 * Deprecated functions
 */

uint8_t SPIClass::send(uint8_t data) {
    this->write(data);
    return 1;
}

uint8_t SPIClass::send(uint8_t *buf, uint32_t len) {
    this->write(buf, len);
    return len;
}


SPIClass SPI(1);
