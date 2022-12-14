/*
 * functions.h
 *
 *  Created on: 29-Jun-2019
 *      Author: poe
 */

#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


void TFT_drawPixel(int16_t x, int16_t y, uint16_t color);
void TFT_fillScreen(uint16_t color);
void TFT_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void TFT_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void TFT_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void TFT_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void TFT_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void TFT_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void TFT_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void TFT_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void TFT_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void TFT_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
void TFT_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);

void TFT_scrollup (uint16_t speed);

void TFT_scrolldown (uint16_t speed);


#endif /* FUNCTIONS_H_ */
