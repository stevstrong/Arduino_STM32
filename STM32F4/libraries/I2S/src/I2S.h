#ifndef _I2S_H
#define _I2S_H

#include <SPI.h>
#ifdef _VSCODE_
#include "..\..\SPI\src\SPI.h"
#endif

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE,
  I2S_PCM_MODE,
} i2s_mode_t;

typedef struct {
    uint8_t ws;
    uint8_t ck;
    uint8_t mck;
    uint8_t sd;
} i2s_pins_t; // __attribute((alligned(4)));

class I2SClass : public SPIClass
{
  public:
    I2SClass(uint8_t port_num, uint8_t sd, uint8_t ws, uint8_t ck);

    I2SClass(uint8_t port_num, uint8_t sd, uint8_t ws, uint8_t ck, uint8_t mck);

    void begin(i2s_mode_t mode, uint32_t sampleRate, uint8_t bitsPerSample);

    void setBuffer(uint16_t *buffer, int bufferSize);

    int getBufferSize();

    uint32_t getDelay();

    uint32_t availableForWrite();

    // void write(int16_t data);

    // void write(int16_t *data, size_t size);

    // I2S_HandleTypeDef handle;

    bool useMck = false;

    i2s_pins_t i2s_pins;

    // DMA_HandleTypeDef dmaHandle;

    uint32_t bufferSize;
    uint16_t *doubleBuffer;
    uint16_t dmaSendSize = 512;

    volatile uint32_t head, tail;
    volatile bool dmaDone;
    // SPIClass &spi;

};

#endif
