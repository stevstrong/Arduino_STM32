#include "STM32F4ADC.h"


/*
    This will read the Vcc and return something useful.
    Polling is being used.
*/
float STM32ADC::readVref()
{
    uint16_t result = adc_read(_dev, 17);
    float vref = (3300.0*result)/4096; // mV
    return vref/1000;
}

/*
    This will read the internal Temperature sensor.
*/
float STM32ADC::readTemp()
{
    uint16_t result = adc_read(_dev, 16);
    //Serial.print("Res: "); Serial.print(result);
    float vSense = (3300.0*result)/4096; // mV
    //Serial.print(", mV: "); Serial.print(Vsense); Serial.print(", temp: ");
    float temperature = ((vSense-v25)/averageSlope) + 25.0; 
    return temperature;
}

/*
    This function will set the number of Pins to sample and which PINS to convert. 
    This uses the IO port numbers and not the ADC channel numbers. Do not confuse. 
*/
void STM32ADC::setPins(const uint8 * pins, uint8 length)
{
    //convert pins to channels.
    uint8 channels[length];
    for (uint8 i = 0; i < length; i++) { //convert the channels from pins to ch.
        channels[i] = PIN_MAP[pins[i]].adc_channel;
    }

    adc_set_reg_sequence(_dev, channels, length);
    if ( length>1 ) adc_set_scan_mode(_dev);
}

/*
    This function is used to setup DMA2 with the ADC. 
    Used in both continuous and scan mode. 
*/
void STM32ADC::setDMA(void * buf, uint16 bufLen, uint8 dual, uint8 continous, uint32 dmaFlags, voidFuncPtr func)
{
    //initialize DMA
    dma_init(DMA2);
    dma_setup_transfer(DMA2, _dev->dmaStream, _dev->dmaChannel, (dual?DMA_SIZE_32BITS:DMA_SIZE_16BITS), &_dev->regs->DR, buf, NULL, dmaFlags);
    dma_set_num_transfers(DMA2, _dev->dmaStream, bufLen);
    dma_set_fifo_flags(DMA2, _dev->dmaStream, 0);
    dma_clear_isr_bits(DMA2, _dev->dmaStream);
    //if there is an int handler to be attached
    if (func != NULL)
        dma_attach_interrupt(DMA2, _dev->dmaStream, func);
    if ( continous )
        adc_dma_continuous(_dev);
    else
        adc_dma_single(_dev);

    enableDMA();
}

/*
    This will set an Analog Watchdog on a channel.
    It must be used with a channel that is being converted.
*/
void STM32ADC::setWD(uint8 channel, uint32 highLimit, uint32 lowLimit, voidFuncPtr func)
{
    adc_awd_set_low_limit(_dev, lowLimit);
    adc_awd_set_high_limit(_dev, highLimit);
    adc_awd_enable_channel(_dev, channel);
    if ( func!=NULL )
        adc_attach_interrupt(_dev, ADC_AWD, func);
    adc_awd_enable(_dev);
}

/*
    check analog watchdog
    Poll the status on the watchdog. This will return and reset the bit.
*/
uint8 STM32ADC::getWDActiveFlag()
{
    return 1;
}
