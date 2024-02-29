/*
See rights and use declaration in License.h
This library has been modified for the Maple Mini.
Includes DMA transfers on DMA1 CH2 and CH3.
*/
#include <Adafruit_ILI9341_STM.h>


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9341_STM::Adafruit_ILI9341_STM(int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX_AS(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT)
{
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
}


void Adafruit_ILI9341_STM::writecommand(uint16_t c)
{
  dc_command();
  cs_clear();
  spiwrite(c);
  dc_data();
}
void Adafruit_ILI9341_STM::writecommand2(uint16_t c)
{
  dc_command();
  cs_clear();
  spiwrite(c>>8);
  spiwrite(c);
  dc_data();
}


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9341_STM::commandList(uint8_t *addr)
{
  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while (numCommands--) {                // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while (numArgs--) {                  //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255) ms = 500;    // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Adafruit_ILI9341_STM::begin(SPIClass & spi, uint32_t freq)
{
  mSPI = spi;
  _freq = freq;
  _safe_freq = (freq>SAFE_FREQ) ? SAFE_FREQ : _freq;
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portSetRegister(_cs);
  cspinmask = digitalPinToBitMask(_cs);
  cs_set(); // deactivate chip
  dcport    = portSetRegister(_dc);
  dcpinmask = digitalPinToBitMask(_dc);

  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));

  // toggle RST low to reset
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }

  uint8_t buf[4];
  ReadCmdDataN(ILI9341_RDID, 4, buf);
  Serial.print("\nRDID:");
  Serial.print(buf[0], HEX);
  Serial.print(buf[1], HEX);
  Serial.print(buf[2], HEX);
  Serial.println(buf[3], HEX);
  /*
  uint8_t x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */
  //if(cmdList) commandList(cmdList);

  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);
  writedata(0x00);
  writedata(0XC1);
  writedata(0X30);

  writecommand(0xED);
  writedata(0x64);
  writedata(0x03);
  writedata(0X12);
  writedata(0X81);

  writecommand(0xE8);
  writedata(0x85);
  writedata(0x00);
  writedata(0x78);

  writecommand(0xCB);
  writedata(0x39);
  writedata(0x2C);
  writedata(0x00);
  writedata(0x34);
  writedata(0x02);

  writecommand(0xF7);
  writedata(0x20);

  writecommand(0xEA);
  writedata(0x00);
  writedata(0x00);

  writecommand(ILI9341_PWCTR1);    //Power control
  writedata(0x23);   //VRH[5:0]

  writecommand(ILI9341_PWCTR2);    //Power control
  writedata(0x10);   //SAP[2:0];BT[3:0]

  writecommand(ILI9341_VMCTR1);    //VCM control
  writedata(0x3e);
  writedata(0x28);

  writecommand(ILI9341_VMCTR2);    //VCM control2
  writedata(0x86);  //--

  writecommand(ILI9341_MADCTL);    // Memory Access Control
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);
  writedata(0x55);

  writecommand(ILI9341_FRMCTR1);
  writedata(0x00);
  writedata(0x18);

  writecommand(ILI9341_DFUNCTR);    // Display Function Control
  writedata(0x08);
  writedata(0x82);
  writedata(0x27);

  writecommand(0xF2);    // 3Gamma Function Disable
  writedata(0x00);

  writecommand(ILI9341_GAMMASET);    //Gamma curve selected
  writedata(0x01);

  writecommand(ILI9341_GMCTRP1);    //Set Gamma
  writedata(0x0F);
  writedata(0x31);
  writedata(0x2B);
  writedata(0x0C);
  writedata(0x0E);
  writedata(0x08);
  writedata(0x4E);
  writedata(0xF1);
  writedata(0x37);
  writedata(0x07);
  writedata(0x10);
  writedata(0x03);
  writedata(0x0E);
  writedata(0x09);
  writedata(0x00);

  writecommand(ILI9341_GMCTRN1);    //Set Gamma
  writedata(0x00);
  writedata(0x0E);
  writedata(0x14);
  writedata(0x03);
  writedata(0x11);
  writedata(0x07);
  writedata(0x31);
  writedata(0xC1);
  writedata(0x48);
  writedata(0x08);
  writedata(0x0F);
  writedata(0x0C);
  writedata(0x31);
  writedata(0x36);
  writedata(0x0F);

  writecommand(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  writecommand(ILI9341_DISPON);    //Display on
  delay(120);
  cs_set();

  _width  = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));

}


void Adafruit_ILI9341_STM::setAddrWindow(uint16_t x0, uint16_t y0,
                                         uint16_t x1, uint16_t y1)
{
  writecommand(ILI9341_CASET); // Column addr set
  spiwrite(x0);
  spiwrite(x1);
  
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite(y0);
  spiwrite(y1);

  writecommand(ILI9341_RAMWR); // write to RAM
  cs_set();
}

void Adafruit_ILI9341_STM::setAddrWindowRead(uint16_t x0, uint16_t y0,
                                         uint16_t x1, uint16_t y1)
{
  writecommand(ILI9341_CASET); // Column addr set
  spiwrite(x0);
  spiwrite(x1);
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite(y0);
  spiwrite(y1);
  writecommand(ILI9341_RAMRD); // write to RAM
}

void Adafruit_ILI9341_STM::pushColors(uint16_t * colorBuffer, uint32_t nr_pixels, uint8_t async)
{
  cs_clear();

  if (async==0) {
    if (nr_pixels>ILI9341_STM_DMA_ON_LIMIT) {
      mSPI.dmaSend(colorBuffer, nr_pixels);
    } else {
      mSPI.write(colorBuffer, nr_pixels);
    }
    cs_set();
  } else {
    mSPI.dmaSendAsync(colorBuffer, nr_pixels, 1);
  }
}

void Adafruit_ILI9341_STM::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x, y, x, y);
  pushColor(color);
}


void Adafruit_ILI9341_STM::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                        uint16_t color)
{
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || h < 1)) return;
  if ((y + h - 1) >= _height)
    h = _height - y;
  if (h < 2 ) {
    drawPixel(x, y, color);
    return;
  }

  setAddrWindow(x, y, x, y + h - 1);

  cs_clear();
  if (h>ILI9341_STM_DMA_ON_LIMIT) {
    mSPI.dmaSend(color, h);
  } else {
    mSPI.write(color, h);
  }
  cs_set();
}


void Adafruit_ILI9341_STM::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                        uint16_t color)
{
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width - x;
  if (w < 2 ) {
    drawPixel(x, y, color);
    return;
  }

  setAddrWindow(x, y, x + w - 1, y);

  cs_clear();
  if (w>ILI9341_STM_DMA_ON_LIMIT) {
    mSPI.dmaSend(color, w);
  } else {
    mSPI.write(color, w);
  }
  cs_set();
}

void Adafruit_ILI9341_STM::fillScreen(uint16_t color)
{
  setAddrWindow(0, 0, _width - 1, _height - 1);
  cs_clear();
  uint32_t nr_bytes = _width * _height;
  while ( nr_bytes>65535 ) {
    nr_bytes -= 65535;
    mSPI.dmaSend(color, (65535));
  }
  mSPI.dmaSend(color, nr_bytes);
  cs_set();
}

// fill a rectangle
void Adafruit_ILI9341_STM::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   uint16_t color)
{
  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height || h < 1 || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width  - x;
  if ((y + h - 1) >= _height) h = _height - y;
  if (w == 1 && h == 1) {
    drawPixel(x, y, color);
    return;
  }

  setAddrWindow(x, y, x + w - 1, y + h - 1);
  cs_clear();
  uint32_t nr_bytes = w * h;
  if ( nr_bytes>ILI9341_STM_DMA_ON_LIMIT ) {
    while ( nr_bytes>65535 ) {
      nr_bytes -= 65535;
      mSPI.dmaSend(color, (65535));
    }
    mSPI.dmaSend(color, nr_bytes);
  } else {
    mSPI.write(color, nr_bytes);
  }
  cs_set();
}

/*
* Draw lines faster by calculating straight sections and drawing them with fastVline and fastHline.
*/
void Adafruit_ILI9341_STM::drawLine(int16_t x0, int16_t y0,
                                    int16_t x1, int16_t y1, uint16_t color)
{
	if ((y0 < 0 && y1 <0) || (y0 > _height && y1 > _height)) return;
	if ((x0 < 0 && x1 <0) || (x0 > _width && x1 > _width)) return;
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;

	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		}
		else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		}
		else {
			drawPixel(x0, y0, color);
		}
		return;
	}
	else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		}
		else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	if (steep) {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine (y0, xbegin, len + 1, color);
					//writeVLine_cont_noCS_noFill(y0, xbegin, len + 1);
				}
				else {
					drawPixel(y0, x0, color);
					//writePixel_cont_noCS(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeVLine_cont_noCS_noFill(y0, xbegin, x0 - xbegin);
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	}
	else {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
					//writeHLine_cont_noCS_noFill(xbegin, y0, len + 1);
				}
				else {
					drawPixel(x0, y0, color);
					//writePixel_cont_noCS(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeHLine_cont_noCS_noFill(xbegin, y0, x0 - xbegin);
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9341_STM::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


// Vertical hardware scrolling
void Adafruit_ILI9341_STM::vertScrollSetArea(uint16_t top, uint16_t bottom)
{
    uint8_t d[6];           // for multi-byte parameters
    d[0] = (uint8_t)(top >> 8);        //top
    d[1] = (uint8_t)(top);
    uint16_t vsarea = HEIGHT - top - bottom;
    d[2] = (uint8_t)(vsarea >> 8); //VSA
    d[3] = (uint8_t)(vsarea);
    d[4] = (uint8_t)(bottom >> 8);        //bottom
    d[5] = (uint8_t)(bottom);
    WriteCmdDataN(ILI9341_VSCRDEF, 6, d);
}



#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341_STM::setRotation(uint8_t m)
{
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
    case 0:
      m = (MADCTL_MX | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      m = (MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 2:
      m = (MADCTL_MY | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
  }
  mSPI.setDataSize(SPI_DATA_SIZE_8BIT);
  writecommand(ILI9341_MADCTL);
  writedata(m);
  cs_set();
  mSPI.setDataSize(SPI_DATA_SIZE_16BIT);
}


void Adafruit_ILI9341_STM::invertDisplay(boolean i)
{
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  cs_set();
}


uint16_t Adafruit_ILI9341_STM::readPixel(int16_t x, int16_t y)
{
  setAddrWindowRead(x, y, x, y);
  uint16_t const reads1 = spiread2();
  uint16_t const reads2 = spiread2();
  uint8_t const r = (uint8_t)(reads1&0xFF);
  uint8_t const g = (uint8_t)(reads2 >> 8);
  uint8_t const b = (uint8_t)(reads2);

  cs_set();
  return color565(r, g, b);
}

uint16_t Adafruit_ILI9341_STM::readPixels(uint16_t *buf, uint16_t num)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
  (void)spiread();             //dummy read
  uint8_t rgb[3];
  // uint16_t len = (x2-x1+1)*(y2-y1+1);
  uint16_t ret = num;
  while (num--) {
    mSPI.read(rgb, 3);
    *buf++ = color565(rgb[0], rgb[1], rgb[2]);
  }
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
  return ret;
}

uint16_t Adafruit_ILI9341_STM::readPixels24(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t *buf)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));

  writecommand(ILI9341_CASET); // Column addr set
  spiwrite2(x1);
  spiwrite2(x2);
  writecommand(ILI9341_PASET); // Row addr set
  spiwrite2(y1);
  spiwrite2(y2);
  writecommand(ILI9341_RAMRD); // read GRAM
  (void)spiread();             //dummy read
  uint16_t len = (x2-x1+1)*(y2-y1+1);
  uint16_t ret = len;
  mSPI.dmaTransfer(buf, buf, len*3);
  cs_set();

  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
  return ret;
}

uint8_t Adafruit_ILI9341_STM::ReadCmdData1(uint8_t c)
{
  writecommand(c);
  uint8_t r = spiread();
  cs_set();
  return r;
}
void Adafruit_ILI9341_STM::ReadCmdDataN(uint8_t cmd, int8_t N, uint8_t * block)
{
    writecommand(cmd);
    readdataN(block, N);
    cs_set();
}
void Adafruit_ILI9341_STM::WriteCmdData1(uint8_t cmd, uint8_t data)
{
    writecommand(cmd);
    writedata(data);
    cs_set();
}

void Adafruit_ILI9341_STM::WriteCmdData2(uint8_t cmd, uint16_t data)
{
    writecommand(cmd);
    writedata((uint8_t)(data>>8));
    writedata((uint8_t)data);
    cs_set();
}
void Adafruit_ILI9341_STM::WriteCmdDataN(uint8_t cmd, int8_t N, uint8_t * block)
{
    writecommand(cmd);
    writedataN(block, N);
    cs_set();
}

uint8_t Adafruit_ILI9341_STM::ReadCmdData1Safe(uint8_t c)
{
  // the SPI clock must be set lower
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
  uint8_t r = ReadCmdData1(c);
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
  return r;
}
void Adafruit_ILI9341_STM::ReadCmdDataNSafe(uint8_t cmd, int8_t N, uint8_t * block)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
    ReadCmdDataN(cmd, N, block);
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
}
void Adafruit_ILI9341_STM::WriteCmdData1Safe(uint8_t cmd, uint8_t data)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
    WriteCmdData1(cmd, data);
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
}
void Adafruit_ILI9341_STM::WriteCmdData2Safe(uint8_t cmd, uint16_t data)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
    WriteCmdData2(cmd, data);
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
}
void Adafruit_ILI9341_STM::WriteCmdDataNSafe(uint8_t cmd, int8_t N, uint8_t * block)
{
  mSPI.beginTransaction(SPISettings(_safe_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_8BIT));
    WriteCmdDataN(cmd, N, block);
  mSPI.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0, SPI_DATA_SIZE_16BIT));
}



/*

 uint16_t Adafruit_ILI9341_STM::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);

 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 uint32_t Adafruit_ILI9341_STM::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!

 dummyclock();
 dummyclock();

 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 */