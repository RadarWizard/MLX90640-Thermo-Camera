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

/*Maximum value the ADC will give*/
#define ADC_MAXIMUM (4096.0f)

/*Resistance between XM and XP*/
#define TOUCHSCREEN_RESISTANCE (252.0f)

//Make RD_Pin go low
#define RD_ACTIVE RD_PORT->BRR = (uint32_t)RD_PIN
//Make RD_Pin go high
#define RD_IDLE RD_PORT->BSRR = (uint32_t)RD_PIN
#define RD_OUTPUT  PIN_OUTPUT(RD_PORT, RD_PIN)

//Make WR_Pin go low
#define WR_ACTIVE WR_PORT->BRR = (uint32_t)WR_PIN
//Make WR_Pin go high
#define WR_IDLE    WR_PORT->BSRR = (uint32_t)WR_PIN
#define WR_OUTPUT  PIN_OUTPUT(WR_PORT, WR_PIN)

//Make CD_PIN go low
#define CD_COMMAND CD_PORT->BRR = (uint32_t)CD_PIN;
//Make CD_PIN go high
#define CD_DATA CD_PORT->BSRR = (uint32_t)CD_PIN;

#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)

//Make CS_PIN go low
#define CS_ACTIVE  CS_PORT->BRR = (uint32_t)CS_PIN
//Make CS_Pin go high
#define CS_IDLE    CS_PORT->BSRR = (uint32_t)CS_PIN
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

#define WriteCmd(x)  { CD_COMMAND; write8(x); CD_DATA; }
#define WriteData(x) { write16(x); }


/************************************************************************************************************/

uint16_t m_width    = WIDTH;
uint16_t m_height   = HEIGHT;

uint16_t TFT_width(void)
{
	return m_width;
}

uint16_t TFT_height(void)
{
	return m_height;
}

void pushColors16b(uint16_t * block, int16_t n, uint8_t first);
void pushColors8b(uint8_t * block, int16_t n, uint8_t first);
void pushColors4n(const uint8_t * block, int16_t n, uint8_t first, uint8_t bigend);
void TFT_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void TFT_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h);

void setReadDir (void);
void setWriteDir (void);

static uint8_t is8347;

uint16_t TFT_color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3);
}

uint16_t readPixel(int16_t x, int16_t y)
{
	uint16_t color;
	readGRAM(x, y, &color, 1, 1);
	return color;
}

static void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, uint8_t first, uint8_t flags);

static void writecmddata(uint16_t cmd, uint16_t dat);

void WriteCmdData(uint16_t cmd, uint16_t dat)
{
	writecmddata(cmd, dat);
}

static inline void WriteCmdParam4(uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);

static void init_table(const void *table, int16_t size);

static void WriteCmdParamN(uint16_t cmd, int8_t N, uint8_t * block);

void pushCommand(uint16_t cmd, uint8_t * block, int8_t N)
{
	WriteCmdParamN(cmd, N, block);
}

static uint16_t read16bits(void);

uint16_t readReg(uint16_t reg, int8_t index);

uint32_t readReg32(uint16_t reg);

uint32_t readReg40(uint16_t reg);

uint16_t m_cursor_y  = 0, m_cursor_x    = 0;
uint8_t m_textsize  = 1;
uint16_t m_textcolor =0xFFFF,  m_textbgcolor = 0x0000;
uint8_t m_wrap      = true;
uint8_t m_rotation  = 0;

#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

uint16_t  _lcd_capable;

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
            h = *block++;
            l = *block++;
        } else {
		    h = *block++;
            l = *block++;
		}
        color = (isbigend) ? (h << 8 | l) :  (l << 8 | h);
        write16(color);
    }
    CS_IDLE;
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
        uint8_t cmd = *p++;
        uint8_t len = *p++;
        if (cmd == TFTLCD_DELAY8)
        {
            delay(len);
            len = 0;
        }
        else
        {
            for (uint8_t i = 0; i < len; i++)
                dat[i] = *p++;
            WriteCmdParamN(cmd, len, dat);
        }
        size -= len + 2;
    }
}


void TFT_reset(void)
{
    setWriteDir();
    //Make TFT control pins outputs
    RD_OUTPUT;
    WR_OUTPUT;
    CD_OUTPUT;
    CS_OUTPUT;
    RESET_OUTPUT;

    //Set initial state of TFT control pins
    CS_IDLE;
    RD_IDLE;
    WR_IDLE;
    //Generate a reset signal for the TFT display
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
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
    const uint8_t *table8_ads = NULL;
    int16_t table_size;
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
        break;
    default:
    	m_width = 0;
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
    uint16_t ret;
    uint8_t msb;

    ret = readReg32(0xD3);      //for ILI9488, 9486, 9340, 9341
    msb = ret >> 8;
    if (msb == 0x93 || msb == 0x94 || msb == 0x98 || msb == 0x77 || msb == 0x16)
        return ret;             //0x9488, 9486, 9340, 9341, 7796
    else
    	return 0;
}

// independent cursor and window registers.   S6D0154, ST7781 increments.  ILI92320/5 do not.
int16_t readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint16_t ret, dummy, _MR = _MW;
    int16_t n = w * h, row = 0, col = 0;
    uint8_t r, g, b;
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
                    ret = TFT_color565(b, g, r);
                else
                    ret = TFT_color565(r, g, b);
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
        setAddrWindow(0, 0, m_width - 1, m_height - 1);
    return 0;
}

void TFT_setRotation(uint8_t r)
{
   uint16_t GS, SS_v, ORG;
   uint8_t val;
   m_rotation = r & 3;           // just perform the operation ourselves on the protected variables
   m_width = (m_rotation & 1) ? HEIGHT : WIDTH;
   m_height = (m_rotation & 1) ? WIDTH : HEIGHT;
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
       _MC = 0x2A, _MP = 0x2B, _MW = 0x2C, _SC = 0x2A, _EC = 0x2A, _SP = 0x2B, _EP = 0x2B;
       WriteCmdParamN(is8347 ? 0x16 : 0x36, 1, &val);
       _lcd_madctl = val;
   }
   // cope with 9320 variants
   else {
       switch (_lcd_ID) {
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

           ORG = (val & 0x20) ? (1 << 3) : 0;
           if (val & 0x08)
               ORG |= 0x1000;  //BGR
           _lcd_madctl = ORG | 0x0030;
           WriteCmdData(0x03, _lcd_madctl);    // set GRAM write direction and BGR=1.
           break;
		}
   }
   if ((m_rotation & 1) && ((_lcd_capable & MV_AXIS) == 0)) {
       uint16_t x;
       x = _MC, _MC = _MP, _MP = x;
       x = _SC, _SC = _SP, _SP = x;    //.kbv check 0139
       x = _EC, _EC = _EP, _EP = x;    //.kbv check 0139
   }
   setAddrWindow(0, 0, m_width - 1, m_height - 1);
   TFT_vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void TFT_drawPixel(int16_t x, int16_t y, uint16_t color)
{
   // MCUFRIEND just plots at edge if you try to write outside of the box:
   if (x < 0 || y < 0 || x >= TFT_width() || y >= TFT_height())
       return;
   setAddrWindow(x, y, x, y);

   WriteCmdData(_MW, color);
}

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
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
    int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
    int16_t vsp;

    if (_lcd_ID == 0x9327) bfa += 32;
    if (offset <= -scrollines || offset >= scrollines) offset = 0; //valid scroll
	vsp = top + offset; // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range

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
    TFT_fillRect(0, 0, m_width, m_height, color);
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
    case 0x9225:                                        //REV is in reg(0x07) like Samsung
    case 0x0154:
        WriteCmdData(0x07, 0x13 | (_lcd_rev << 2));     //.kbv kludge
        break;
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

void  TFT_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	TFT_fillRect(x, y, 1, h, color);
}
void  TFT_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	TFT_fillRect(x, y, w, 1, color);
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
            TFT_drawPixel(y0, x0, color);
        } else {
            TFT_drawPixel(x0, y0, color);
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
        if(y0 > y1)
        	_swap_int16_t(y0, y1);
        TFT_drawFastVLine(x0, y0, y1 - y0 + 1, color);
    } else if(y0 == y1){
        if(x0 > x1)
        	_swap_int16_t(x0, x1);
        TFT_drawFastHLine(x0, y0, x1 - x0 + 1, color);
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

    TFT_drawPixel(x0, y0 + r, color);
    TFT_drawPixel(x0, y0 - r, color);
    TFT_drawPixel(x0 + r, y0, color);
    TFT_drawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        TFT_drawPixel(x0 + x, y0 + y, color);
        TFT_drawPixel(x0 - x, y0 + y, color);
        TFT_drawPixel(x0 + x, y0 - y, color);
        TFT_drawPixel(x0 - x, y0 - y, color);
        TFT_drawPixel(x0 + y, y0 + x, color);
        TFT_drawPixel(x0 - y, y0 + x, color);
        TFT_drawPixel(x0 + y, y0 - x, color);
        TFT_drawPixel(x0 - y, y0 - x, color);
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
            TFT_drawPixel(x0 + x, y0 + y, color);
            TFT_drawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            TFT_drawPixel(x0 + x, y0 - y, color);
            TFT_drawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            TFT_drawPixel(x0 - y, y0 + x, color);
            TFT_drawPixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            TFT_drawPixel(x0 - y, y0 - x, color);
            TFT_drawPixel(x0 - x, y0 - y, color);
        }
    }
}

void TFT_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    TFT_drawFastVLine(x0, y0-r, 2*r+1, color);
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
            if(corners & 1) TFT_drawFastVLine(x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) TFT_drawFastVLine(x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) TFT_drawFastVLine(x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) TFT_drawFastVLine(x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void TFT_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    TFT_drawFastHLine(x, y, w, color);
    TFT_drawFastHLine(x, y+h-1, w, color);
    TFT_drawFastVLine(x, y, h, color);
    TFT_drawFastVLine(x+w-1, y, h, color);
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
    if (end > m_width)
        end = m_width;
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > m_height)
        end = m_height;
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
    if (!(_lcd_capable & MIPI_DCS_REV1))
        setAddrWindow(0, 0, m_width - 1, m_height - 1);
}


void TFT_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    TFT_drawFastHLine(x+r  , y    , w-2*r, color); // Top
    TFT_drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
    TFT_drawFastVLine(x    , y+r  , h-2*r, color); // Left
    TFT_drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
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
        TFT_drawFastHLine(a, y0, b-a+1, color);
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
        TFT_drawFastHLine(a, y, b-a+1, color);
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
        TFT_drawFastHLine(a, y, b-a+1, color);
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

	if( w > 0 && h > 0)
	{
		if(size > 1) {
			xo16 = xo;
			yo16 = yo;
		}
		/*Fill rectangle above character*/
		TFT_fillRect(x, y - gfxFont->yAdvance + gfxFont->yoffset,  glyph->xAdvance, gfxFont->yAdvance + yo - gfxFont->yoffset , m_textbgcolor);
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
		TFT_fillRect(x, y + gfxFont->yoffset,  glyph->xAdvance, (yo +h) - gfxFont->yoffset  , m_textbgcolor); //PRW Test

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
						TFT_drawPixel(x+xo+xx, y+yo+yy, color);
					}else{
						TFT_drawPixel(x+xo+xx, y+yo+yy, bg);
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
	}else{/*No bitmap to display so fill with background*/
		TFT_fillRect(x, y - gfxFont->yAdvance + gfxFont->yoffset,  glyph->xAdvance, gfxFont->yAdvance, m_textbgcolor);
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
			GFXglyph *glyph = &gfxFont->glyph[c - first];
			uint16_t   w     = glyph->width,
					   h     = glyph->height;
			if((w > 0) && (h > 0)) { // Is there an associated bitmap?
				int16_t xo = (int8_t)glyph->xOffset; // sic
				if(m_wrap && ((m_cursor_x + m_textsize * (xo + w)) > m_width)) {
					m_cursor_x  = 0;
					m_cursor_y += (int16_t)m_textsize * (uint16_t)gfxFont->yAdvance;
				}
			}
			drawChar(m_cursor_x, m_cursor_y, c, m_textcolor, m_textbgcolor, m_textsize);
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
                GFXglyph *glyph = &gfxFont->glyph[c - first];
                uint8_t gw = glyph->width,
                        gh = glyph->height,
                        xa = glyph->xAdvance;
                int8_t  xo = glyph->xOffset,
                        yo = glyph->yOffset;
                if(m_wrap && ((*x+(((int16_t)xo+gw)*m_textsize)) > m_width)) {
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
            if(m_wrap && ((*x + m_textsize * 6) > m_width)) { // Off right?
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

    int16_t minx = m_width, miny = m_height, maxx = -1, maxy = -1;

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
     if (getRotation() & 1) maxscroll = TFT_width();
     else maxscroll = TFT_height();
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
	if (getRotation() & 1) maxscroll = TFT_width();
	     else maxscroll = TFT_height();
	for (uint16_t i = 1; i <= maxscroll; i++)
	{
		TFT_vertScroll(0, maxscroll, 0 - (int16_t)i);
		if (speed < 655) delay(speed*100);
		else HAL_Delay(speed);
	}
}

/***************************//*
 * Make a measurement on touchscreen.
 * The pressure is actually the resistance between the resistive planes
 * when the screen is touched. When touched with a stylus the resistance
 * is ~400ohms but when touched with a finger is ~200ohms
 */
uint16_t TS_Measure(ADC_HandleTypeDef *hadc, TSPoint *ts)
{
uint32_t    ret;
float		p1, p2, rt, x_cor, y_cor;

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

  x_cor = (uint16_t)HAL_ADC_GetValue(hadc);		/*x-axis coordinate for resistive screen*/
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

  y_cor = (uint16_t)HAL_ADC_GetValue(hadc);	/*y coordinate for resistive screen*/

  /*Measure pressure*/
  PIN_OUTPUT(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);
  PIN_OUTPUT(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);
  PIN_ANALOGUE(LCD_TS_XM_GPIO_Port, LCD_TS_XM_Pin);
  PIN_ANALOGUE(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);

  PIN_HIGH(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);
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

  p1 = (float)HAL_ADC_GetValue(hadc);

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

  p2 = (float)HAL_ADC_GetValue(hadc);

  /* Calculate the resistance between the resistive touch screen layers when pressed
   * P2 can be zero when there is on touch. This could lead to a divide by zero error.
   * Therefore check for p2 being zero and if it is report resistance as 1Mohm*/
  if (p2 >= 1.0f)
  {
	  rt = (p1 / p2 - 1.0) * TOUCHSCREEN_RESISTANCE * (float)x_cor / ADC_MAXIMUM;
  }else{
	  rt = 65535.0f;
  }

  ts->pressure = (uint16_t)rt;

  /*Calculate position of touch from ADC measurements*/
  y_cor = (uint16_t)(-0.1309f * y_cor + 508.7f);
  x_cor = (uint16_t)(-0.09718f * x_cor  + 354.9f);

  /*Return coordinates based on screen rotation*/
  switch (m_rotation) {
  case 0:                    //PORTRAIT:
	  ts->xcor  = WIDTH - x_cor;
	  ts->ycor  = y_cor;
      break;
  case 1:                    //LANDSCAPE: 90 degrees
	  ts->xcor  = y_cor;
	  ts->ycor  = x_cor;
      break;
  case 2:                    //PORTRAIT_REV: 180 degrees
	  ts->xcor  = x_cor;
	  ts->ycor  = HEIGHT - y_cor;
      break;
  case 3:                    //LANDSCAPE_REV: 270 degrees
	  ts->xcor  = HEIGHT - y_cor;
	  ts->ycor  = WIDTH -x_cor;
      break;
  default:
	  ts->xcor  = x_cor;
	  ts->ycor  = y_cor;
      break;
  }

  /*Ensure pins are configured correctly for display use. S*/
  PIN_OUTPUT(LCD_TS_YP_GPIO_Port, LCD_TS_YP_Pin);
  PIN_OUTPUT(LCD_TS_YM_GPIO_Port, LCD_TS_YM_Pin);
  PIN_OUTPUT(LCD_TS_XP_GPIO_Port, LCD_TS_XP_Pin);
  PIN_OUTPUT(LCD_TS_XM_GPIO_Port, LCD_TS_XM_Pin);

  return 0;
}

