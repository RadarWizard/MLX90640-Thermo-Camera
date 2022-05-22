/**
  ******************************************************************************
  * @file    hx8347g.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-May-2014
  * @brief   This file includes the LCD driver for HX8347G LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tft.h"
//#include "stm32l4xx_hal.h"
#include "string.h"
#include "functions.h"
#include "user_setting.h"
#include "stdlib.h"

/********************************************** NO CHNAGES AFTER THIS ************************************************/


void PIN_LOW (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void PIN_HIGH (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void PIN_INPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {	.Mode = GPIO_MODE_INPUT,
											.Pull = GPIO_NOPULL,
											.Pin = GPIO_Pin};

	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void PIN_OUTPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void PIN_ANALOGUE (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


#define RD_ACTIVE  PIN_LOW(RD_PORT, RD_PIN)
#define RD_IDLE    PIN_HIGH(RD_PORT, RD_PIN)
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)
#define WR_ACTIVE  PIN_LOW(WR_PORT, WR_PIN)
#define WR_IDLE    PIN_HIGH(WR_PORT, WR_PIN)
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)
//#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
//Make CD_PIN go low
#define CD_COMMAND CD_PORT->BRR = (uint32_t)CD_PIN;
//Make CD_PIN go high
#define CD_DATA CD_PORT->BSRR = (uint32_t)CD_PIN;

//#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN)
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN)
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)


#define WR_ACTIVE2  {WR_ACTIVE; WR_ACTIVE;}
#define WR_ACTIVE4  {WR_ACTIVE2; WR_ACTIVE2;}
#define WR_ACTIVE8  {WR_ACTIVE4; WR_ACTIVE4;}
#define RD_ACTIVE2  {RD_ACTIVE; RD_ACTIVE;}
#define RD_ACTIVE4  {RD_ACTIVE2; RD_ACTIVE2;}
#define RD_ACTIVE8  {RD_ACTIVE4; RD_ACTIVE4;}
#define RD_ACTIVE16 {RD_ACTIVE8; RD_ACTIVE8;}
#define WR_IDLE2  {WR_IDLE; WR_IDLE;}
#define WR_IDLE4  {WR_IDLE2; WR_IDLE2;}
#define RD_IDLE2  {RD_IDLE; RD_IDLE;}
#define RD_IDLE4  {RD_IDLE2; RD_IDLE2;}

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }         //PWLW=TWRL=50ns
#define RD_STROBE RD_IDLE, RD_ACTIVE, RD_ACTIVE, RD_ACTIVE   //PWLR=TRDL=150ns
//#define RD_STROBE {RD_IDLE; RD_ACTIVE; RD_ACTIVE; RD_ACTIVE;}   //PWLR=TRDL=150ns


#define write8(x)     { write_8(x); WRITE_DELAY; WR_STROBE; }
#define write16(x)    { uint8_t h = (x)>>8, l = x; write8(h); write8(l); }
#define READ_8(dst)   { RD_STROBE; READ_DELAY; dst = read_8(); RD_IDLE; RD_IDLE; } // read 250ns after RD_ACTIVE goes low
#define READ_16(dst)  { uint8_t hi; READ_8(hi); READ_8(dst); dst |= (hi << 8); }

#define CTL_INIT()   { RD_OUTPUT; WR_OUTPUT; CD_OUTPUT; CS_OUTPUT; RESET_OUTPUT; }
#define WriteCmd(x)  { CD_COMMAND; write8(x); CD_DATA; }
#define WriteData(x) { write16(x); }


/************************************************************************************************************/

uint16_t _width    = WIDTH;
uint16_t _height   = HEIGHT;

uint16_t width(void)
{ return _width; }

uint16_t height(void)
{ return _height; }

void pushColors16b(uint16_t * block, int16_t n, uint8_t first);
void pushColors8b(uint8_t * block, int16_t n, uint8_t first);
void pushColors4n(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend);
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h);

void setReadDir (void);
void setWriteDir (void);

static uint8_t done_reset, is8347, is9797;

static uint8_t color565_to_r(uint16_t color) {
    return ((color & 0xF800) >> 8);  // transform to rrrrrxxx
}
static uint8_t color565_to_g(uint16_t color) {
    return ((color & 0x07E0) >> 3);  // transform to ggggggxx
}
static uint8_t color565_to_b(uint16_t color) {
    return ((color & 0x001F) << 3);  // transform to bbbbbxxx
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3); }
uint16_t readPixel(int16_t x, int16_t y) { uint16_t color; readGRAM(x, y, &color, 1, 1); return color; }

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags);

static void write24(uint16_t color);

static void writecmddata(uint16_t cmd, uint16_t dat);

void WriteCmdData(uint16_t cmd, uint16_t dat) { writecmddata(cmd, dat); }

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);

static void init_table(const void *table, int16_t size);

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block);

void pushCommand(uint16_t cmd, uint8_t * block, int8_t N) { WriteCmdParamN(cmd, N, block); }

static uint16_t read16bits(void);

uint16_t readReg(uint16_t reg, int8_t index);

uint32_t readReg32(uint16_t reg);

uint32_t readReg40(uint16_t reg);

uint16_t m_cursor_y  =0, m_cursor_x    = 0;
uint8_t m_textsize  = 1;
uint16_t m_textcolor =0xFFFF,  m_textbgcolor = 0x0000;
uint8_t m_wrap      = true;
uint8_t m_rotation  = 0;

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr))

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

uint16_t  _lcd_xor, _lcd_capable;

uint16_t _lcd_ID, _lcd_rev, _lcd_madctl, _lcd_drivOut, _MC, _MP, _MW, _SC, _EC, _SP, _EP;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags)
{
    uint16_t color;
    uint8_t h, l;
	uint8_t isconst = flags & 1;
	uint8_t isbigend = (flags & 2) != 0;
    CS_ACTIVE;
    if (first) {
        WriteCmd(cmd);
    }

    if (!isconst && !isbigend) {
        uint16_t *block16 = (uint16_t*)block;
        while (n-- > 0) {
            color = *block16++;
            write16(color);
        }
    } else

    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
		    h = (*block++);
            l = (*block++);
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
        if (is9797) write24(color); else
        write16(color);
    }
    CS_IDLE;
}

static void write24(uint16_t color)
{
    uint8_t r = color565_to_r(color);
    uint8_t g = color565_to_g(color);
    uint8_t b = color565_to_b(color);
    write8(r);
    write8(g);
    write8(b);
}


static void writecmddata(uint16_t cmd, uint16_t dat)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    WriteData(dat);
    CS_IDLE;
}



static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    while (N-- > 0) {
        uint8_t u8 = *block++;
        write8(u8);
        if (N && is8347) {
            cmd++;
            WriteCmd(cmd);
        }
    }
    CS_IDLE;
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
    uint8_t d[4];
    d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4;
    WriteCmdParamN(cmd, 4, d);
}

#define TFTLCD_DELAY 0xFFFF
#define TFTLCD_DELAY8 0x7F
static void init_table(const void *table, int16_t size)
{

    uint8_t *p = (uint8_t *) table, dat[24];            //R61526 has GAMMA[22]

    while (size > 0)
    {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8)
        {
            delay(len);
            len = 0;
        }
        else
        {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = pgm_read_byte(p++);
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}


static void init_table16(const void *table, int16_t size)
{
    uint16_t *p = (uint16_t *) table;
    while (size > 0) {
        uint16_t cmd = pgm_read_word(p++);
        uint16_t d = pgm_read_word(p++);
        if (cmd == TFTLCD_DELAY)
            delay(d);
        else {
			writecmddata(cmd, d);                      //static function
        }
        size -= 2 * sizeof(int16_t);
    }
}



void TFT_reset(void)
{
    done_reset = 1;
    setWriteDir();
    CTL_INIT();
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
	WriteCmdData(0xB0, 0x0000);   //R61520 needs this to read ID
}

static uint16_t read16bits(void)
{
    uint16_t ret;
    uint8_t lo;
    READ_8(ret);
    READ_8(lo);
    return (ret << 8) | lo;
}

uint16_t readReg(uint16_t reg, int8_t index)
{
    uint16_t ret;
    if (!done_reset)
        TFT_reset();
    CS_ACTIVE;
    WriteCmd(reg);
    setReadDir();
    delay(1);    //1us should be adequate
    //    READ_16(ret);
    do {
    	ret = read16bits();
    }while (--index >= 0);  //need to test with SSD1963
    RD_IDLE;
    CS_IDLE;
    setWriteDir();
    return ret;
}

uint32_t readReg32(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t l = readReg(reg, 1);
    return ((uint32_t) h << 16) | (l);
}

uint32_t readReg40(uint16_t reg)
{
    uint16_t h = readReg(reg, 0);
    uint16_t m = readReg(reg, 1);
    uint16_t l = readReg(reg, 2);
    return ((uint32_t) h << 24) | (m << 8) | (l >> 8);
}


void TFT_init(uint16_t ID)
{
    int16_t *p16;               //so we can "write" to a const protected variable.
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
    _lcd_xor = 0;
    switch (_lcd_ID = ID) {

    case 0x9488:
        _lcd_capable = AUTO_READINC | MIPI_DCS_REV1 | MV_AXIS | READ_24BITS;
        static const uint8_t ILI9488_regValues_max[]  = {        // Atmel MaxTouch
            0xC0, 2, 0x10, 0x10,        //Power Control 1 [0E 0E]
            0xC1, 1, 0x41,      //Power Control 2 [43]
            0xC5, 4, 0x00, 0x22, 0x80, 0x40,    //VCOM  Control 1 [00 40 00 40]
            0x36, 1, 0x68,      //Memory Access [00]
            0xB0, 1, 0x00,      //Interface     [00]
            0xB1, 2, 0xB0, 0x11,        //Frame Rate Control [B0 11]
            0xB4, 1, 0x02,      //Inversion Control [02]
            0xB6, 3, 0x02, 0x02, 0x3B,  // Display Function Control [02 02 3B] .kbv NL=480
            0xB7, 1, 0xC6,      //Entry Mode      [06]
            0x3A, 1, 0x55,      //Interlace Pixel Format [XX]
            0xF7, 4, 0xA9, 0x51, 0x2C, 0x82,    //Adjustment Control 3 [A9 51 2C 82]
        };
        table8_ads = ILI9488_regValues_max, table_size = sizeof(ILI9488_regValues_max);
        p16 = (int16_t *) & height;
        *p16 = 480;
        p16 = (int16_t *) & width;
        *p16 = 320;
        break;
    default:
        p16 = (int16_t *) & width;
        *p16 = 0;       //error value for width
        break;
    }
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0);
    if (table8_ads != NULL) {
        static const uint8_t reset_off[]  = {
            0x01, 0,            //Soft Reset
            TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
            0x28, 0,            //Display Off
            0x3A, 1, 0x55,      //Pixel read=565, write=565.
        };
        static const uint8_t wake_on[]  = {
			0x11, 0,            //Sleep Out
            TFTLCD_DELAY8, 150,
            0x29, 0,            //Display On
        };
		init_table(&reset_off, sizeof(reset_off));
	    init_table(table8_ads, table_size);   //can change PIXFMT
		init_table(&wake_on, sizeof(wake_on));
    }
    TFT_setRotation(0);             //PORTRAIT
    TFT_invertDisplay(false);
}




uint16_t TFT_readID(void)
{
    uint16_t ret, ret2;
    uint8_t msb;
    ret = readReg(0,0);           //forces a reset() if called before begin()
    if (ret == 0x5408)          //the SPFD5408 fails the 0xD3D3 test.
        return 0x5408;
    if (ret == 0x5420)          //the SPFD5420 fails the 0xD3D3 test.
        return 0x5420;
    if (ret == 0x8989)          //SSD1289 is always 8989
        return 0x1289;
    ret = readReg(0x67,0);        //HX8347-A
    if (ret == 0x4747)
        return 0x8347;
//#if defined(SUPPORT_1963) && USING_16BIT_BUS
    ret = readReg32(0xA1);      //SSD1963: [01 57 61 01]
    if (ret == 0x6101)
        return 0x1963;
    if (ret == 0xFFFF)          //R61526: [xx FF FF FF]
        return 0x1526;          //subsequent begin() enables Command Access
//    if (ret == 0xFF00)          //R61520: [xx FF FF 00]
//        return 0x1520;          //subsequent begin() enables Command Access
//#endif
	ret = readReg40(0xBF);
	if (ret == 0x8357)          //HX8357B: [xx 01 62 83 57 FF]
        return 0x8357;
	if (ret == 0x9481)          //ILI9481: [xx 02 04 94 81 FF]
        return 0x9481;
    if (ret == 0x1511)          //?R61511: [xx 02 04 15 11] not tested yet
        return 0x1511;
    if (ret == 0x1520)          //?R61520: [xx 01 22 15 20]
        return 0x1520;
    if (ret == 0x1526)          //?R61526: [xx 01 22 15 26]
        return 0x1526;
    if (ret == 0x1581)          //R61581:  [xx 01 22 15 81]
        return 0x1581;
    if (ret == 0x1400)          //?RM68140:[xx FF 68 14 00] not tested yet
        return 0x6814;
    ret = readReg32(0xD4);
    if (ret == 0x5310)          //NT35310: [xx 01 53 10]
        return 0x5310;
    ret = readReg32(0xD7);
    if (ret == 0x8031)          //weird unknown from BangGood [xx 20 80 31] PrinceCharles
        return 0x8031;
    ret = readReg40(0xEF);      //ILI9327: [xx 02 04 93 27 FF]
    if (ret == 0x9327)
        return 0x9327;
    ret = readReg32(0xFE) >> 8; //weird unknown from BangGood [04 20 53]
    if (ret == 0x2053)
        return 0x2053;
    uint32_t ret32 = readReg32(0x04);
    msb = ret32 >> 16;
    ret = ret32;
    if (msb == 0x00 && ret == 0x8000) { //HX8357-D [xx 00 80 00]
#if 1
        uint8_t cmds[] = {0xFF, 0x83, 0x57};
        pushCommand(0xB9, cmds, 3);
        msb = readReg(0xD0,0);
        if (msb == 0x99) return 0x0099; //HX8357-D from datasheet
        if (msb == 0x90)        //HX8357-C undocumented
#endif
            return 0x9090;      //BIG CHANGE: HX8357-D was 0x8357
    }
    if (ret == 0x1526)          //R61526 [xx 06 15 26] if I have written NVM
        return 0x1526;          //subsequent begin() enables Command Access
	if (ret == 0x89F0)          //ST7735S: [xx 7C 89 F0]
        return 0x7735;
	if (ret == 0x8552)          //ST7789V: [xx 85 85 52]
        return 0x7789;
    if (ret == 0xAC11)          //?unknown [xx 61 AC 11]
        return 0xAC11;
    ret32 = readReg32(0xD3);      //[xx 91 63 00]
    ret = ret32 >> 8;
    if (ret == 0x9163) return ret;
    ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    msb = ret >> 8;
    if (msb == 0x93 || msb == 0x94 || msb == 0x98 || msb == 0x77 || msb == 0x16)
        return ret;             //0x9488, 9486, 9340, 9341, 7796
    if (ret == 0x00D3 || ret == 0xD3D3)
        return ret;             //16-bit write-only bus
	return readReg(0,0);          //0154, 7783, 9320, 9325, 9335, B505, B509
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b, tmp;
    if (!is8347 && (_lcd_capable & MIPI_DCS_REV1)) // HX8347 uses same register
        _MR = 0x2E;
    if (_lcd_ID == 0x1602) _MR = 0x2E;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    while (n > 0) {
        if (!(_lcd_capable & MIPI_DCS_REV1)) {
            WriteCmdData(_MC, x + col);
            WriteCmdData(_MP, y + row);
        }
        CS_ACTIVE;
        WriteCmd(_MR);
        setReadDir();
        if (_lcd_capable & READ_NODUMMY) {
            ;
        } else if ((_lcd_capable & MIPI_DCS_REV1) || _lcd_ID == 0x1289) {
            READ_8(r);
        } else {
            READ_16(dummy);
        }
		if (_lcd_ID == 0x1511) READ_8(r);   //extra dummy for R61511
        while (n)
        {
            if (_lcd_capable & READ_24BITS)
            {
                READ_8(r);
                READ_8(g);
                READ_8(b);
                if (_lcd_capable & READ_BGR)
                    ret = color565(b, g, r);
                else
                    ret = color565(r, g, b);
            } else
            {
                READ_16(ret);
                if (_lcd_capable & READ_LOWHIGH)
                    ret = (ret >> 8) | (ret << 8);
                if (_lcd_capable & READ_BGR)
                    ret = (ret & 0x07E0) | (ret >> 11) | (ret << 11);
            }
            *block++ = ret;
            n--;
            if (!(_lcd_capable & AUTO_READINC))
                break;
        }
        if (++col >= w) {
            col = 0;
            if (++row >= h)
                row = 0;
        }
        RD_IDLE;
        CS_IDLE;
        setWriteDir();
    }
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

void TFT_setRotation(uint8_t r)
{
   uint16_t GS, SS_v, ORG, REV = _lcd_rev;
   uint8_t val, d[3];
   m_rotation = r & 3;           // just perform the operation ourselves on the protected variables
   _width = (m_rotation & 1) ? HEIGHT : WIDTH;
   _height = (m_rotation & 1) ? WIDTH : HEIGHT;
   switch (m_rotation) {
   case 0:                    //PORTRAIT:
       val = 0x48;             //MY=0, MX=1, MV=0, ML=0, BGR=1
       break;
   case 1:                    //LANDSCAPE: 90 degrees
       val = 0x28;             //MY=0, MX=0, MV=1, ML=0, BGR=1
       break;
   case 2:                    //PORTRAIT_REV: 180 degrees
       val = 0x98;             //MY=1, MX=0, MV=0, ML=1, BGR=1
       break;
   case 3:                    //LANDSCAPE_REV: 270 degrees
       val = 0xF8;             //MY=1, MX=1, MV=1, ML=1, BGR=1
       break;
   }
   if (_lcd_capable & INVERT_GS)
       val ^= 0x80;
   if (_lcd_capable & INVERT_SS)
       val ^= 0x40;
   if (_lcd_capable & INVERT_RGB)
       val ^= 0x08;
   if (_lcd_capable & MIPI_DCS_REV1) {
       if (_lcd_ID == 0x6814) {  //.kbv my weird 0x9486 might be 68140
           GS = (val & 0x80) ? (1 << 6) : 0;   //MY
           SS_v = (val & 0x40) ? (1 << 5) : 0;   //MX
           val &= 0x28;        //keep MV, BGR, MY=0, MX=0, ML=0
           d[0] = 0;
           d[1] = GS | SS_v | 0x02;      //MY, MX
           d[2] = 0x3B;
           WriteCmdParamN(0xB6, 3, d);
           goto common_MC;
       }
       else if (_lcd_ID == 0x1963 || _lcd_ID == 0x9481 || _lcd_ID == 0x1511) {
           if (val & 0x80)
               val |= 0x01;    //GS
           if ((val & 0x40))
               val |= 0x02;    //SS
           if (_lcd_ID == 0x1963) val &= ~0xC0;
           if (_lcd_ID == 0x9481) val &= ~0xD0;
           if (_lcd_ID == 0x1511) {
               val &= ~0x10;   //remove ML
               val |= 0xC0;    //force penguin 180 rotation
           }
           goto common_MC;
      }
       else if (is8347) {
           _MC = 0x02, _MP = 0x06, _MW = 0x22, _SC = 0x02, _EC = 0x04, _SP = 0x06, _EP = 0x08;
           if (_lcd_ID == 0x0065) {             //HX8352-B
               val |= 0x01;    //GS=1
               if ((val & 0x10)) val ^= 0xD3;  //(ML) flip MY, MX, ML, SS, GS
               if (r & 1) _MC = 0x82, _MP = 0x80;
               else _MC = 0x80, _MP = 0x82;
           }
           if (_lcd_ID == 0x5252) {             //HX8352-A
               val |= 0x02;   //VERT_SCROLLON
               if ((val & 0x10)) val ^= 0xD4;  //(ML) flip MY, MX, SS. GS=1
           }
			goto common_BGR;
       }
     common_MC:
       _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
     common_BGR:
       WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
       _lcd_madctl = val;
   }
   // cope with 9320 variants
   else {
       switch (_lcd_ID) {
#if defined(SUPPORT_9225)
       case 0x9225:
           _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
           _MC = 0x20, _MP = 0x21, _MW = 0x22;
           GS = (val & 0x80) ? (1 << 9) : 0;
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           WriteCmdData(0x01, GS | SS_v | 0x001C);       // set Driver Output Control
           goto common_ORG;
#endif
#if defined(SUPPORT_0139) || defined(SUPPORT_0154)
#ifdef SUPPORT_0139
       case 0x0139:
           _SC = 0x46, _EC = 0x46, _SP = 0x48, _EP = 0x47;
           goto common_S6D;
#endif
#ifdef SUPPORT_0154
       case 0x0154:
           _SC = 0x37, _EC = 0x36, _SP = 0x39, _EP = 0x38;
           goto common_S6D;
#endif
         common_S6D:
           _MC = 0x20, _MP = 0x21, _MW = 0x22;
           GS = (val & 0x80) ? (1 << 9) : 0;
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           // S6D0139 requires NL = 0x27,  S6D0154 NL = 0x28
           WriteCmdData(0x01, GS | SS_v | ((_lcd_ID == 0x0139) ? 0x27 : 0x28));
           goto common_ORG;
#endif
       case 0x5420:
       case 0x7793:
       case 0x9326:
		case 0xB509:
           _MC = 0x200, _MP = 0x201, _MW = 0x202, _SC = 0x210, _EC = 0x211, _SP = 0x212, _EP = 0x213;
           GS = (val & 0x80) ? (1 << 15) : 0;
			uint16_t NL;
			NL = ((432 / 8) - 1) << 9;
           if (_lcd_ID == 0x9326 || _lcd_ID == 0x5420) NL >>= 1;
           WriteCmdData(0x400, GS | NL);
           goto common_SS;
       default:
           _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
           GS = (val & 0x80) ? (1 << 15) : 0;
           WriteCmdData(0x60, GS | 0x2700);    // Gate Scan Line (0xA700)
         common_SS:
           SS_v = (val & 0x40) ? (1 << 8) : 0;
           WriteCmdData(0x01, SS_v);     // set Driver Output Control
         common_ORG:
           ORG = (val & 0x20) ? (1 << 3) : 0;
#ifdef SUPPORT_8230
           if (_lcd_ID == 0x8230) {    // UC8230 has strange BGR and READ_BGR behaviour
               if (m_rotation == 1 || m_rotation == 2) {
                   val ^= 0x08;        // change BGR bit for LANDSCAPE and PORTRAIT_REV
               }
           }
#endif
           if (val & 0x08)
               ORG |= 0x1000;  //BGR
           _lcd_madctl = ORG | 0x0030;
           WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
           break;
#ifdef SUPPORT_1289
       case 0x1289:
           _MC = 0x4E, _MP = 0x4F, _MW = 0x22, _SC = 0x44, _EC = 0x44, _SP = 0x45, _EP = 0x46;
           if (m_rotation & 1)
               val ^= 0xD0;    // exchange Landscape modes
           GS = (val & 0x80) ? (1 << 14) : 0;    //called TB (top-bottom), CAD=0
           SS_v = (val & 0x40) ? (1 << 9) : 0;   //called RL (right-left)
           ORG = (val & 0x20) ? (1 << 3) : 0;  //called AM
           _lcd_drivOut = GS | SS_v | (REV << 13) | 0x013F;      //REV=0, BGR=0, MUX=319
           if (val & 0x08)
               _lcd_drivOut |= 0x0800; //BGR
           WriteCmdData(0x01, _lcd_drivOut);   // set Driver Output Control
           if (is9797) WriteCmdData(0x11, ORG | 0x4C30); else  // DFM=2, DEN=1, WM=1, TY=0
           WriteCmdData(0x11, ORG | 0x6070);   // DFM=3, EN=0, TY=1
           break;
#endif
		}
   }
   if ((m_rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
       uint16_t x;
       x = _MC, _MC = _MP, _MP = x;
       x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
       x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
   }
   setAddrWindow(0, 0, width() - 1, height() - 1);
   TFT_vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void TFT_drawPixel(int16_t x, int16_t y, uint16_t color)
{
   // MCUFRIEND just plots at edge if you try to write outside of the box:
   if (x < 0 || y < 0 || x >= width() || y >= height())
       return;
   setAddrWindow(x, y, x, y);
   if (is9797) { CS_ACTIVE; WriteCmd(_MW); write24(color); CS_IDLE;} else
   WriteCmdData(_MW, color);
}

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (m_rotation == 2) y += OFFSET_9327, y1 += OFFSET_9327;
	    if (m_rotation == 3) x += OFFSET_9327, x1 += OFFSET_9327;
   }
#endif
#if 1
   if (_lcd_ID == 0x1526 && (m_rotation & 1)) {
		int16_t dx = x1 - x, dy = y1 - y;
		if (dy == 0) { y1++; }
		else if (dx == 0) { x1 += dy; y1 -= dy; }
   }
#endif
   if (_lcd_capable & MIPI_DCS_REV1) {
       WriteCmdParam4(_SC, x >> 8, x, x1 >> 8, x1);   //Start column instead of _MC
       WriteCmdParam4(_SP, y >> 8, y, y1 >> 8, y1);   //
       if (is8347 && _lcd_ID == 0x0065) {             //HX8352-B has separate _MC, _SC
           uint8_t d[2];
           d[0] = x >> 8; d[1] = x;
           WriteCmdParamN(_MC, 2, d);                 //allows !MV_AXIS to work
           d[0] = y >> 8; d[1] = y;
           WriteCmdParamN(_MP, 2, d);
       }
   } else {
       WriteCmdData(_MC, x);
       WriteCmdData(_MP, y);
       if (!(x == x1 && y == y1)) {  //only need MC,MP for drawPixel
           if (_lcd_capable & XSA_XEA_16BIT) {
               if (m_rotation & 1)
                   y1 = y = (y1 << 8) | y;
               else
                   x1 = x = (x1 << 8) | x;
           }
           WriteCmdData(_SC, x);
           WriteCmdData(_SP, y);
           WriteCmdData(_EC, x1);
           WriteCmdData(_EP, y1);
       }
   }
}

void TFT_vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
#if defined(OFFSET_9327)
	if (_lcd_ID == 0x9327) {
	    if (m_rotation == 2 || m_rotation == 3) top += OFFSET_9327;
    }
#endif
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;
    int16_t sea = top;
	if (_lcd_ID == 0x9327) bfa += 32;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    sea = top + scrollines - 1;
    if (_lcd_capable & MIPI_DCS_REV1) {
        uint8_t d[6];           // for multi-byte parameters
        d[0] = top >> 8;        //TFA
        d[1] = top;
        d[2] = scrollines >> 8; //VSA
        d[3] = scrollines;
        d[4] = bfa >> 8;        //BFA
        d[5] = bfa;
        WriteCmdParamN(is8347 ? 0x0E : 0x33, 6, d);
		d[0] = vsp >> 8;        //VSP
        d[1] = vsp;
        WriteCmdParamN(is8347 ? 0x14 : 0x37, 2, d);
		if (is8347) {
		    d[0] = (offset != 0) ? (_lcd_ID == 0x8347 ? 0x02 : 0x08) : 0;
			WriteCmdParamN(_lcd_ID == 0x8347 ? 0x18 : 0x01, 1, d);  //HX8347-D
		} else if (offset == 0 && (_lcd_capable & MIPI_DCS_REV1)) {
			WriteCmdParamN(0x13, 0, NULL);    //NORMAL i.e. disable scroll
		}
		return;
    }
    // cope with 9320 style variants:
    switch (_lcd_ID) {
    case 0x7783:
        WriteCmdData(0x61, _lcd_rev);   //!NDL, !VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
#ifdef SUPPORT_0139
    case 0x0139:
        WriteCmdData(0x07, 0x0213 | (_lcd_rev << 2));  //VLE1=1, GON=1, REV=x, D=3
        WriteCmdData(0x41, vsp);  //VL# check vsp
        break;
#endif
#if defined(SUPPORT_0154) || defined(SUPPORT_9225)  //thanks tongbajiel
    case 0x9225:
	case 0x0154:
        WriteCmdData(0x31, sea);        //SEA
        WriteCmdData(0x32, top);        //SSA
        WriteCmdData(0x33, vsp - top);  //SST
        break;
#endif
#ifdef SUPPORT_1289
    case 0x1289:
        WriteCmdData(0x41, vsp);        //VL#
        break;
#endif
	case 0x5420:
    case 0x7793:
	case 0x9326:
	case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //VLE, REV
        WriteCmdData(0x404, vsp);       //VL#
        break;
    default:
        // 0x6809, 0x9320, 0x9325, 0x9335, 0xB505 can only scroll whole screen
        WriteCmdData(0x61, (1 << 1) | _lcd_rev);        //!NDL, VLE, REV
        WriteCmdData(0x6A, vsp);        //VL#
        break;
    }
}




void pushColors16b(uint16_t * block, int16_t n, uint8_t first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void pushColors8b(uint8_t * block, int16_t n, uint8_t first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void pushColors4n(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}


void TFT_fillScreen(uint16_t color)
{
    TFT_fillRect(0, 0, _width, _height, color);
}

void TFT_invertDisplay(uint8_t i)
{
    uint8_t val;
    _lcd_rev = ((_lcd_capable & REV_SCREEN) != 0) ^ i;
    if (_lcd_capable & MIPI_DCS_REV1) {
        if (is8347) {
            // HX8347D: 0x36 Panel Characteristic. REV_Panel
            // HX8347A: 0x36 is Display Control 10
            if (_lcd_ID == 0x8347 || _lcd_ID == 0x5252) // HX8347-A, HX5352-A
			    val = _lcd_rev ? 6 : 2;       //INVON id bit#2,  NORON=bit#1
            else val = _lcd_rev ? 8 : 10;     //HX8347-D, G, I: SCROLLON=bit3, INVON=bit1
            // HX8347: 0x01 Display Mode has diff bit mapping for A, D
            WriteCmdParamN(0x01, 1, &val);
        } else
            WriteCmdParamN(_lcd_rev ? 0x21 : 0x20, 0, NULL);
        return;
    }
    // cope with 9320 style variants:
    switch (_lcd_ID) {
#ifdef SUPPORT_0139
    case 0x0139:
#endif
    case 0x9225:                                        //REV is in reg(0x07) like Samsung
    case 0x0154:
        WriteCmdData(0x07, 0x13 | (_lcd_rev << 2));     //.kbv kludge
        break;
#ifdef SUPPORT_1289
    case 0x1289:
        _lcd_drivOut &= ~(1 << 13);
        if (_lcd_rev)
            _lcd_drivOut |= (1 << 13);
        WriteCmdData(0x01, _lcd_drivOut);
        break;
#endif
	case 0x5420:
    case 0x7793:
    case 0x9326:
	case 0xB509:
        WriteCmdData(0x401, (1 << 1) | _lcd_rev);       //.kbv kludge VLE
        break;
    default:
        WriteCmdData(0x61, _lcd_rev);
        break;
    }
}

void  drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	TFT_fillRect(x, y, 1, h, color);
}
void  drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	TFT_fillRect(x, y, w, 1, color);
}

void writePixel(int16_t x, int16_t y, uint16_t color)
{
    TFT_drawPixel(x, y, color);
}

void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
        uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            writePixel(y0, x0, color);
        } else {
            writePixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}


void TFT_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    if(x0 == x1){
        if(y0 > y1) _swap_int16_t(y0, y1);
        drawFastVLine(x0, y0, y1 - y0 + 1, color);
    } else if(y0 == y1){
        if(x0 > x1) _swap_int16_t(x0, x1);
        drawFastHLine(x0, y0, x1 - x0 + 1, color);
    } else {
        writeLine(x0, y0, x1, y1, color);
    }
}

void TFT_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    writePixel(x0  , y0+r, color);
    writePixel(x0  , y0-r, color);
    writePixel(x0+r, y0  , color);
    writePixel(x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        writePixel(x0 + x, y0 + y, color);
        writePixel(x0 - x, y0 + y, color);
        writePixel(x0 + x, y0 - y, color);
        writePixel(x0 - x, y0 - y, color);
        writePixel(x0 + y, y0 + x, color);
        writePixel(x0 - y, y0 + x, color);
        writePixel(x0 + y, y0 - x, color);
        writePixel(x0 - y, y0 - x, color);
    }
}

void TFT_drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
            writePixel(x0 + x, y0 + y, color);
            writePixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            writePixel(x0 + x, y0 - y, color);
            writePixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            writePixel(x0 - y, y0 + x, color);
            writePixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            writePixel(x0 - y, y0 - x, color);
            writePixel(x0 - x, y0 - y, color);
        }
    }
}

void TFT_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    drawFastVLine(x0, y0-r, 2*r+1, color);
    TFT_fillCircleHelper(x0, y0, r, 3, 0, color);
}

void TFT_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) drawFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) drawFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) drawFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) drawFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void TFT_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    drawFastHLine(x, y, w, color);
    drawFastHLine(x, y+h-1, w, color);
    drawFastVLine(x, y, h, color);
    drawFastVLine(x+w-1, y, h, color);
}

void TFT_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    CS_ACTIVE;
    WriteCmd(_MW);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (h-- > 0) {
        end = w;
        do {
            write8(hi);
            write8(lo);
        } while (--end != 0);
    }
    CS_IDLE;
    if (!(_lcd_capable & MIPI_DCS_REV1) || ((_lcd_ID == 0x1526) && (m_rotation & 1)))
        setAddrWindow(0, 0, width() - 1, height() - 1);
}


void TFT_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    drawFastHLine(x+r  , y    , w-2*r, color); // Top
    drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
    drawFastVLine(x    , y+r  , h-2*r, color); // Left
    drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    TFT_drawCircleHelper(x+r    , y+r    , r, 1, color);
    TFT_drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
    TFT_drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    TFT_drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}


void TFT_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    TFT_fillRect(x+r, y, w-2*r, h, color);
    // draw four corners
    TFT_fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    TFT_fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}


void TFT_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    TFT_drawLine(x0, y0, x1, y1, color);
    TFT_drawLine(x1, y1, x2, y2, color);
    TFT_drawLine(x2, y2, x0, y0, color);
}


void TFT_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{

    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        drawFastHLine(a, y0, b-a+1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        drawFastHLine(a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        drawFastHLine(a, y, b-a+1, color);
    }
}


/********************************* TESTS  *********************************************/

void TFT_testFillScreen()
{
    TFT_fillScreen(BLACK);
    TFT_fillScreen(RED);
    TFT_fillScreen(GREEN);
    TFT_fillScreen(BLUE);
    TFT_fillScreen(BLACK);
}

void TFT_testLines(uint16_t color)
{
    int           x1, y1, x2, y2,
                  w = width(),
                  h = height();

    TFT_fillScreen(BLACK);

    x1 = y1 = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = 0;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6) TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6) TFT_drawLine(x1, y1, x2, y2, color);

}

void TFT_testFastLines(uint16_t color1, uint16_t color2)
{
    int           x, y, w = width(), h = height();

    TFT_fillScreen(BLACK);
    for (y = 0; y < h; y += 5) drawFastHLine(0, y, w, color1);
    for (x = 0; x < w; x += 5) drawFastVLine(x, 0, h, color2);
}

void TFT_testRects(uint16_t color) {
    int           n, i, i2,
                  cx = width()  / 2,
                  cy = height() / 2;

    TFT_fillScreen(BLACK);
    n     = min(width(), height());
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        TFT_drawRect(cx - i2, cy - i2, i, i, color);
    }

}

void TFT_testFilledRects(uint16_t color1, uint16_t color2)
{
    int           n, i, i2,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n = min(width(), height());
    for (i = n; i > 0; i -= 6) {
        i2    = i / 2;

        TFT_fillRect(cx - i2, cy - i2, i, i, color1);

        TFT_drawRect(cx - i2, cy - i2, i, i, color2);
    }
}

void TFT_testFilledCircles(uint8_t radius, uint16_t color)
{
    int x, y, w = width(), h = height(), r2 = radius * 2;

    TFT_fillScreen(BLACK);
    for (x = radius; x < w; x += r2) {
        for (y = radius; y < h; y += r2) {
            TFT_fillCircle(x, y, radius, color);
        }
    }

}

void TFT_testCircles(uint8_t radius, uint16_t color)
{
    int           x, y, r2 = radius * 2,
                        w = width()  + radius,
                        h = height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            TFT_drawCircle(x, y, radius, color);
        }
    }

}

void TFT_testTriangles() {
    int           n, i, cx = width()  / 2 - 1,
                        cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n     = min(cx, cy);
    for (i = 0; i < n; i += 5) {
        TFT_drawTriangle(
            cx    , cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            color565(0, 0, i));
    }

}

void TFT_testFilledTriangles() {
    int           i, cx = width()  / 2 - 1,
                     cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    for (i = min(cx, cy); i > 10; i -= 5) {
        TFT_fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         color565(0, i, i));
        TFT_drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         color565(i, i, 0));
    }
}

void TFT_testRoundRects() {
    int           w, i, i2, red, step,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    w     = min(width(), height());
    red = 0;
    step = (256 * 6) / w;
    for (i = 0; i < w; i += 6) {
        i2 = i / 2;
        red += step;
        TFT_drawRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(red, 0, 0));
    }

}

void TFT_testFilledRoundRects() {
    int           i, i2, green, step,
                  cx = width()  / 2 - 1,
                  cy = height() / 2 - 1;

    TFT_fillScreen(BLACK);
    green = 256;
    step = (256 * 6) / min(width(), height());
    for (i = min(width(), height()); i > 20; i -= 6) {
        i2 = i / 2;
        green -= step;
        TFT_fillRoundRect(cx - i2, cy - i2, i, i, i / 8, color565(0, green, 0));
    }

}



void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size)
{
	// Character is assumed previously filtered by write() to eliminate
	// newlines, returns, non-printable characters, etc.  Calling
	// drawChar() directly with 'bad' characters of font may cause mayhem!

	c -= gfxFont->first;
	GFXglyph *glyph  = &(gfxFont->glyph[c]);
	uint8_t  *bitmap = &gfxFont->bitmap[glyph->bitmapOffset];

	uint8_t  w  = glyph->width,
			 h  = glyph->height;
	int8_t   xo = glyph->xOffset,
			 yo = glyph->yOffset;
	uint8_t  xx, yy, bits = 0, bit = 0;
	int16_t  xo16 = 0, yo16 = 0;

	if(size > 1) {
		xo16 = xo;
		yo16 = yo;
	}
	/*Fill rectangle above character*/
	TFT_fillRect(x, y - gfxFont->yAdvance + 5,  glyph->xAdvance, gfxFont->yAdvance + yo - 5 , m_textbgcolor); //PRW Test
	/*Fill gap to left of character*/
	if(xo > 0)
	{
		TFT_fillRect(x, y + yo, xo , h, m_textbgcolor); //PRW Test
	}
	/*Fill gap to right of character*/
	if(w-xo < glyph->xAdvance)
	{
		TFT_fillRect(x+xo+w, y +yo, glyph->xAdvance - xo -w , h, m_textbgcolor); //PRW Test
	}
	/*Fill gap beneath character*/
	TFT_fillRect(x, y + 5,  glyph->xAdvance, (yo +h) - 5  , m_textbgcolor); //PRW Test

	if(size == 1)
	{
		for(yy=0; yy<h; yy++) {
			for(xx=0; xx<w; xx++) {
				if(!(bit++ & 7)) {
					bits = *bitmap;
					bitmap++;
				}

				if(bits & 0x80)
				{
					writePixel(x+xo+xx, y+yo+yy, color);
				}else{
					writePixel(x+xo+xx, y+yo+yy, bg);
				}
				bits <<= 1;
			}
		}
	}else{
		for(yy=0; yy<h; yy++) {
			for(xx=0; xx<w; xx++) {
				if(!(bit++ & 7)) {
					bits = *bitmap;
					bitmap++;
				}

				if(bits & 0x80)
				{
					TFT_fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size,
					  size, size, color);
				}else{
					TFT_fillRect(x+(xo16+xx)*size, y+(yo16+yy)*size,
					  size, size, bg);
				}
				bits <<= 1;
			}
		}
	}
}
/**************************************************************************/
/*!
    @brief  Print one byte/character of data, used to support print()
    @param  c  The 8-bit ascii character to write
*/
/**************************************************************************/
size_t TFT_write(uint8_t c)
{
	if(c == '\n') {
		m_cursor_x  = 0;
		m_cursor_y += (int16_t)m_textsize * (uint8_t)gfxFont->yAdvance;
	} else if(c != '\r')
	{
		uint8_t first = gfxFont->first;
		if((c >= first) && (c <= (uint8_t)gfxFont->last))
		{
			GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c - first]);
			uint16_t   w     = glyph->width,
					   h     = glyph->height;
			if((w > 0) && (h > 0)) { // Is there an associated bitmap?
				int16_t xo = (int8_t)glyph->xOffset; // sic
				if(m_wrap && ((m_cursor_x + m_textsize * (xo + w)) > _width)) {
					m_cursor_x  = 0;
					m_cursor_y += (int16_t)m_textsize * (uint16_t)gfxFont->yAdvance;
				}
				/*PRW Line to erase background*/
//				TFT_fillRect(m_cursor_x, m_cursor_y - gfxFont->yAdvance + 2, m_textsize * (xo + (uint16_t)glyph->xAdvance), m_textsize * (gfxFont->yAdvance - 1) , m_textbgcolor); //PRW Test
				drawChar(m_cursor_x, m_cursor_y, c, m_textcolor, m_textbgcolor, m_textsize);
			}
			m_cursor_x += (uint16_t)glyph->xAdvance * (int16_t)m_textsize;
		}
	}
    return 1;
}


/**************************************************************************/
/*!
    @brief Set the font to display when print()ing, either custom or default
    @param  f  The GFXfont object, if NULL use built in 6x8 font
*/
/**************************************************************************/
void TFT_setFont(const GFXfont *f) {
    gfxFont = (GFXfont *)f;
}


/**************************************************************************/
/*!
    @brief    Helper to determine size of a character with current font/size.
       Broke this out as it's used by both the - and RAM-resident getTextBounds() functions.
    @param    c     The ascii character in question
    @param    x     Pointer to x location of character
    @param    y     Pointer to y location of character
    @param    minx  Minimum clipping value for X
    @param    miny  Minimum clipping value for Y
    @param    maxx  Maximum clipping value for X
    @param    maxy  Maximum clipping value for Y
*/
/**************************************************************************/
void charBounds(char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy)
{

    if(gfxFont) {

        if(c == '\n') { // Newline?
            *x  = 0;    // Reset x to zero, advance y by one line
            *y += m_textsize * gfxFont->yAdvance;
        } else if(c != '\r') { // Not a carriage return; is normal char
            uint8_t first = gfxFont->first,
                    last  = gfxFont->last;
            if((c >= first) && (c <= last)) { // Char present in this font?
                GFXglyph *glyph = &(((GFXglyph *)pgm_read_pointer(
                  &gfxFont->glyph))[c - first]);
                uint8_t gw = glyph->width,
                        gh = glyph->height,
                        xa = glyph->xAdvance;
                int8_t  xo = glyph->xOffset,
                        yo = glyph->yOffset;
                if(m_wrap && ((*x+(((int16_t)xo+gw)*m_textsize)) > _width)) {
                    *x  = 0; // Reset x to zero, advance y by one line
                    *y += m_textsize * gfxFont->yAdvance;
                }
                int16_t ts = (int16_t)m_textsize,
                        x1 = *x + xo * ts,
                        y1 = *y + yo * ts,
                        x2 = x1 + gw * ts - 1,
                        y2 = y1 + gh * ts - 1;
                if(x1 < *minx) *minx = x1;
                if(y1 < *miny) *miny = y1;
                if(x2 > *maxx) *maxx = x2;
                if(y2 > *maxy) *maxy = y2;
                *x += xa * ts;
            }
        }

    } else { // Default font

        if(c == '\n') {                     // Newline?
            *x  = 0;                        // Reset x to zero,
            *y += m_textsize * 8;             // advance y one line
            // min/max x/y unchaged -- that waits for next 'normal' character
        } else if(c != '\r') {  // Normal char; ignore carriage returns
            if(m_wrap && ((*x + m_textsize * 6) > _width)) { // Off right?
                *x  = 0;                    // Reset x to zero,
                *y += m_textsize * 8;         // advance y one line
            }
            int x2 = *x + m_textsize * 6 - 1, // Lower-right pixel of char
                y2 = *y + m_textsize * 8 - 1;
            if(x2 > *maxx) *maxx = x2;      // Track max x, y
            if(y2 > *maxy) *maxy = y2;
            if(*x < *minx) *minx = *x;      // Track min x, y
            if(*y < *miny) *miny = *y;
            *x += m_textsize * 6;             // Advance x one char
        }
    }
}

/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       The current cursor X
    @param    y       The current cursor Y
    @param    x1      The boundary X coordinate, set by function
    @param    y1      The boundary Y coordinate, set by function
    @param    w      The boundary width, set by function
    @param    h      The boundary height, set by function
*/
/**************************************************************************/
void getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
    uint8_t c; // Current character

    *x1 = x;
    *y1 = y;
    *w  = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while((c = *str++))
        charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);

    if(maxx >= minx) {
        *x1 = minx;
        *w  = maxx - minx + 1;
    }
    if(maxy >= miny) {
        *y1 = miny;
        *h  = maxy - miny + 1;
    }
}


void TFT_printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str)
{
	TFT_setFont(f);
	m_textcolor = txtcolor;
	m_textsize = (txtsize > 0) ? txtsize : 1;
	TFT_setCursor(0, row);
	while (*str)
	{
		TFT_write(*str++);
	}
}

void TFT_printstr (uint8_t *str)
{
	while(*str)
	{
		TFT_write(*str++);
	}
}

void TFT_setTextWrap(uint8_t w) { m_wrap = w; }

void TFT_setTextColor (uint16_t color)
{
	m_textcolor = color;
}

void TFT_setTextbgColor (uint16_t color)
{
	m_textbgcolor = color;
}

void TFT_setTextSize (uint8_t size)
{
	m_textsize = size;
}

void TFT_setCursor(int16_t x, int16_t y) { m_cursor_x = x; m_cursor_y = y; }

uint8_t getRotation (void)
{
	return m_rotation;
}

void TFT_scrollup (uint16_t speed)
{
     uint16_t maxscroll;
     if (getRotation() & 1) maxscroll = width();
     else maxscroll = height();
     for (uint16_t i = 1; i <= maxscroll; i++)
     {
          TFT_vertScroll(0, maxscroll, i);
         if (speed < 655) delay(speed*100);
         else HAL_Delay(speed);
     }

}

void TFT_scrolldown (uint16_t speed)
{
	uint16_t maxscroll;
	if (getRotation() & 1) maxscroll = width();
	     else maxscroll = height();
	for (uint16_t i = 1; i <= maxscroll; i++)
	{
		TFT_vertScroll(0, maxscroll, 0 - (int16_t)i);
		if (speed < 655) delay(speed*100);
		else HAL_Delay(speed);
	}
}

/***************************//*
 * Make a measurement on touchscreen
 */
uint16_t TS_Measure(ADC_HandleTypeDef *hadc, TSPoint *ts)
{
uint32_t    ret;
TSPoint	retval;

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /*Sample touchscreen over 320 pixel axis*/

  /*XM and XP will already be an output so no need to configure it*/
  PIN_ANALOGUE(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);
  PIN_INPUT(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);

  /*XM will already be high so no need to configure it*/
  PIN_LOW(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);

  sConfig.Channel = LCD_TS_YP_ADC_CHAN;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  ret = HAL_ADC_Start(hadc);
  if(ret != HAL_OK)
  {
	  Error_Handler();
  }

  ret = HAL_ADC_PollForConversion(hadc, 20);
  if(ret != HAL_OK)
  {
	  Error_Handler();
  }

  ts->ycor = (uint16_t)HAL_ADC_GetValue(hadc);
  ts->ycor = (uint16_t)(-0.09718f * (float)ts->ycor + 354.9f);
  /*Make XP high before making it an output*/
  PIN_HIGH(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);

  /*Sample touchscreen over 480 pixel axis*/
  PIN_OUTPUT(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);
  PIN_OUTPUT(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);
  PIN_ANALOGUE(LCD_TS_XM_GPIO_Port, LCD_TS_XM_Pin);
  PIN_INPUT(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);

  /*XM will already be high so no need to configure it*/
  PIN_HIGH(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);
  PIN_LOW(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);

  sConfig.Channel = LCD_TS_XM_ADC_CHAN;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  ret = HAL_ADC_Start(hadc);
  if(ret != HAL_OK)
  {
	  Error_Handler();
  }

  ret = HAL_ADC_PollForConversion(hadc, 20);
  if(ret != HAL_OK)
  {
	  Error_Handler();
  }

  ts->xcor = (uint16_t)HAL_ADC_GetValue(hadc);

  ts->xcor = (uint16_t)(-0.1309f * (float)ts->xcor + 508.7f);
  /*Ensure pins are configured correctly for display use. S*/
  PIN_OUTPUT(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);
  PIN_OUTPUT(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);
  PIN_OUTPUT(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);
  PIN_OUTPUT(LCD_TS_XM_GPIO_Port, LCD_TS_XM_Pin);

  return 0;
}

