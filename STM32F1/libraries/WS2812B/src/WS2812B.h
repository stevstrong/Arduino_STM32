/*--------------------------------------------------------------------
  The WS2812B library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  It is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  See <http://www.gnu.org/licenses/>.
  --------------------------------------------------------------------*/

#ifndef WS2812B_H
#define WS2812B_H

#include <Arduino.h>
#include <SPI.h>

class WS2812B {
public:
  // Constructor: number of LEDs
  WS2812B(uint16_t number_of_leds, SPIClass * _spi = &SPI);
  ~WS2812B();
  void
    begin(void),
    show(void),
    setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b),
 //   setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w),
    setPixelColor(uint16_t n, uint32_t c),
    setBrightness(uint8_t),
    clear(),
	  updateLength(uint16_t n);
  uint8_t
//   *getPixels(void) const,
    getBrightness(void) const { return brightness - 1; };
  uint16_t
    numPixels(void) const { return numLEDs; };
  static uint32_t
    Color(uint8_t r, uint8_t g, uint8_t b),
    Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
 // uint32_t
 //   getPixelColor(uint16_t n) const;
  inline bool
    canShow(void) { return (micros() - endTime) >= 100L; }

private:
  boolean
    begun;         // true if begin() previously called
  uint16_t
    numLEDs,       // Number of RGB LEDs in strip
    numBytes;      // Size of 'pixels' buffer
	
  uint8_t
    brightness,
   *pixels,        // Holds the current LED color values, which the external API calls interact with 9 bytes per pixel + start + end empty bytes
   *doubleBuffer,	 // Holds the start of the double buffer (1 buffer for async DMA transfer and one for the API interaction.
    rOffset,       // Index of red byte within each 3- or 4-byte pixel
    gOffset,       // Index of green byte
    bOffset,       // Index of blue byte
    wOffset;       // Index of white byte (same as rOffset if no white)
  uint32_t
    endTime;       // Latch timing reference
  SPIClass * spi;
};


#endif // WS2812B_H
