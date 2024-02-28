/*
Inspired from: https://github.com/danieleff/STM32GENERIC/blob/master/STM32/libraries/I2S/src/I2S.cpp
*/

#include <I2S.h>

#include "util.h"
//------------------------------------------------------------------------------
//  Auxiliary functions
//------------------------------------------------------------------------------
static void disable_pwm(uint8_t pin)
{
    const timer_info * i = &timer_map[pin];
    const timer_dev * t_dev = timer_devices[i->index];
    if (i->index && t_dev) {
        timer_set_mode(t_dev, i->channel, TIMER_DISABLED);
    }
}
//------------------------------------------------------------------------------
static void configure_gpios(i2s_pins_t &pins, uint8_t as_master)
{
    disable_pwm(pins.ws);
    disable_pwm(pins.ck);
    disable_pwm(pins.sd);
    if (pins.mck >= 0) disable_pwm(pins.mck);

    if (as_master) {
        gpio_set_mode(pins.ws, GPIO_AF_OUTPUT_PP);		
        gpio_set_mode(pins.ck, GPIO_AF_OUTPUT_PP);
        gpio_set_mode(pins.sd, GPIO_AF_OUTPUT_PP);
        if (pins.mck >= 0) gpio_set_mode(pins.mck, GPIO_AF_OUTPUT_PP);
    } else {
        gpio_set_mode(pins.ws, GPIO_INPUT_FLOATING);
        gpio_set_mode(pins.ck, GPIO_INPUT_FLOATING);
        gpio_set_mode(pins.sd, GPIO_INPUT_FLOATING);
        gpio_set_mode(pins.mck, GPIO_INPUT_FLOATING);
    }

	gpio_set_af_mode(pins.ws, GPIO_AFMODE_SPI3_5);
	gpio_set_af_mode(pins.ck, GPIO_AFMODE_SPI3_5);
	gpio_set_af_mode(pins.sd, GPIO_AFMODE_SPI3_5);
	if (pins.mck >= 0) gpio_set_af_mode(pins.mck, GPIO_AFMODE_SPI3_5);
}

//------------------------------------------------------------------------------
I2SClass::I2SClass(uint8_t port_num, uint8_t ws, uint8_t sd, uint8_t ck)
: SPIClass (port_num)
{
    i2s_pins.ws = ws;
    i2s_pins.sd = sd;
    i2s_pins.ck = ck;
    i2s_pins.mck = -1;
}

//------------------------------------------------------------------------------
I2SClass::I2SClass(uint8_t port_num, uint8_t ws, uint8_t sd, uint8_t ck, uint8_t mck)
: SPIClass (port_num)
{
    i2s_pins.ws = ws;
    i2s_pins.sd = sd;
    i2s_pins.ck = ck;
    i2s_pins.mck = mck;
}   

//------------------------------------------------------------------------------
void I2SClass::setBuffer(uint16_t *buffer, int bufferSize)
{
    this->doubleBuffer = buffer;
    this->bufferSize = bufferSize;
}

//------------------------------------------------------------------------------
int I2SClass::getBufferSize() {
    return bufferSize;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void i2s_peripheral_disable(spi_dev_t *spi_dev)
{
    spi_dev->regs->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
}
void i2s_peripheral_enable(spi_dev_t *spi_dev)
{
    spi_dev->regs->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void i2s_set_standard(spi_dev_t *spi_dev, uint32_t standard)
{
    uint32_t i2scfgreg = spi_dev->regs->I2SCFGR;
    i2scfgreg &= ~(SPI_I2SCFGR_I2SSTD);
    if (standard == I2S_PHILIPS_MODE) {
        standard = SPI_I2SCFGR_I2SSTD_PHILLIPS;
    } else if (standard == I2S_LEFT_JUSTIFIED_MODE) {
        standard = SPI_I2SCFGR_I2SSTD_LSB;
    } else if (standard == I2S_RIGHT_JUSTIFIED_MODE) {
        standard = SPI_I2SCFGR_I2SSTD_MSB;
    } else if (standard == I2S_PCM_MODE) {
        standard = SPI_I2SCFGR_I2SSTD_PCM;
    }
    i2scfgreg |= standard; // set standard
    spi_dev->regs->I2SCFGR = i2scfgreg;
}

void i2s_set_mode(spi_dev_t *spi_dev, uint32_t mode)
{
    uint32_t i2scfgreg = spi_dev->regs->I2SCFGR;
    i2scfgreg &= ~(SPI_I2SCFGR_I2SCFG);
    i2scfgreg |= mode; // set standard
    spi_dev->regs->I2SCFGR = i2scfgreg;
}
//------------------------------------------------------------------------------
typedef struct {
    uint32_t sample_rate;
    uint16_t plli2s_n;
    uint8_t plli2s_r;
    uint8_t i2s_div;
    uint8_t i2s_odd;
} i2s_config_t;
#define CFG_NUM 7
const i2s_config_t i2s_config_table[CFG_NUM] = {
    {8000, 256, 5, 12, 1},
    {16000, 213, 2, 13, 0},
    {22050, 429, 4, 9, 1},
    {32000, 213, 2, 6, 1},
    {44100, 271, 2, 6, 0},
    {48000, 258, 3, 3, 1},
    {96000, 344, 2, 3, 1},
};

//------------------------------------------------------------------------------
void I2SClass::begin(i2s_mode_t mode, uint32_t sampleRate, uint8_t bitsPerSample)
{
    spi_dev_t *spi_dev = _currentSetting->spi_d;
    spi_init(spi_dev);
    spi_peripheral_disable(spi_dev);
    i2s_peripheral_disable(spi_dev);

    configure_gpios(i2s_pins, 1);
    _currentSetting->state = SPI_STATE_READY;

    // set up clocks
    // RCC->CR &= ~(RCC_CR_PLLI2SON); // Disable PLLI2S

    // Configure PLLI2S
    // This register is used to configure the PLLI2S clock outputs according to the formulas:
    // • f(VCO[out]) = f(VCO[in]) × (PLLI2SN / PLLI2SM)
    // • f(I2SxCLK) = f(VCO[out]) / PLLI2SR
    // PLLI2SR should be chosen [3, 6, 9]  for Fs dividable by 3
    // 
    // When the master clock is generated (MCKOE in the SPI_I2SPR register is set):
    // FS = f(I2SxCLK) / [(16*2)*((2*I2SDIV)+ODD)*8)] when the channel frame is 16-bit wide
    // FS = f(I2SxCLK) / [(32*2)*((2*I2SDIV)+ODD)*4)] when the channel frame is 32-bit wide
    // When the master clock is disabled (MCKOE bit cleared):
    // FS = f(I2SxCLK) / [(16*2)*((2*I2SDIV)+ODD))] when the channel frame is 16-bit wide
    // FS = f(I2SxCLK) / [(32*2)*((2*I2SDIV)+ODD))] when the channel frame is 32-bit wide

    // take values from the table
    const i2s_config_t * i2s_cfg;
    uint8_t i;
    for (i = 0; i < CFG_NUM; i ++) {
        if (i2s_config_table[i].sample_rate == sampleRate) {
            // take this config
            i2s_cfg = &i2s_config_table[i];
            break;
        }
    }
    if (i == CFG_NUM) {
        Serial.println("*** ERROR: Could not find configuration settings for the given sample rate ***");
        return;
    }
    // set I2S prescaller register
    spi_dev->regs->I2SPR = ((1 << 9) | (i2s_cfg->i2s_odd << 8) | i2s_cfg->i2s_div);
    i2s_set_standard(spi_dev, mode);
    i2s_set_mode(spi_dev, (SPI_I2SCFGR_I2SMOD_I2S | SPI_I2SCFGR_I2SCFG_MASTER_TX));

    // setup RCC registers
    RCC->PLLI2SCFGR =  (i2s_cfg->plli2s_r << 28) | (i2s_cfg->plli2s_n << 6) | 8;
    RCC->CR |= RCC_CR_PLLI2SON; // Enable PLLI2S

   // spi_peripheral_enable(spi_dev);
    i2s_peripheral_enable(spi_dev);

    // configure DMA to send data from buffer in circular mode
    // dmaSend(static_buffer, sizeof(static_buffer), DMA_CIRC_MODE);
}

#if 0
void I2SClass::write(int16_t data)
{
    while((head + 1) % bufferSize == tail % bufferSize);

    doubleBuffer[head % bufferSize] = data;
    head++;

    if (dmaDone && (uint32_t)(head - tail) > dmaSendSize) {
        i2sDma = this;
        dmaDone = false;
        HAL_I2S_Transmit_DMA(&handle, (uint16_t*)(doubleBuffer + (tail % bufferSize)), dmaSendSize);
    }

}

void I2SClass::write(int16_t *data, size_t samples) {
    for(size_t i=0; i<samples; i++) {
        write(data[i]);
    }
}

uint32_t I2SClass::availableForWrite() {
    return bufferSize - (head - tail);
}

uint32_t I2SClass::getDelay() {
    return dmaSendSize;
}
#endif
