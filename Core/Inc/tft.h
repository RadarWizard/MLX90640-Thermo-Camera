/**
  ******************************************************************************
  * @file    hx8347g.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-May-2014
  * @brief   This file contains all the functions prototypes for the hx8347g.c
  *          driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HX8347G_H
#define __HX8347G_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "fonts.h"

typedef struct{
 	uint16_t	xcor;
 	uint16_t	ycor;
 	uint16_t	pressure;
}TSPoint;

   
#define true	1
#define false	0

#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#define min(a, b) (((a) < (b)) ? (a) : (b))

void TFT_init(uint16_t ID);
void TFT_reset(void);
uint16_t TFT_readID(void);

void TFT_setRotation(uint8_t r);
void TFT_invertDisplay(uint8_t i);
void TFT_vertScroll(int16_t top, int16_t scrollines, int16_t offset);

void TFT_setFont(const GFXfont *f);
void TFT_setTextWrap(uint8_t w);
void TFT_setTextColor (uint16_t color);
void TFT_setTextbgColor (uint16_t color);
void TFT_setTextSize (uint8_t size);
void TFT_setCursor(int16_t x, int16_t y);
void TFT_printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);
void TFT_printstr (uint8_t *str);
uint16_t TFT_width(void);
uint16_t TFT_height(void);
void  TFT_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void  TFT_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
uint16_t TFT_color565(uint8_t r, uint8_t g, uint8_t b);


uint16_t TS_Measure(ADC_HandleTypeDef *hadc, TSPoint *ts);


#ifdef __cplusplus
}
#endif

#endif /* __HX8347G_H */

