/*
 * tft_test.c
 *
 *  Created on: 3 Jul 2022
 *      Author: Peter
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tft.h"
#include "string.h"
#include "functions.h"
#include "user_setting.h"
#include "stdlib.h"

extern
void TFT_testFillScreen(void)
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
                  w = TFT_width(),
                  h = TFT_height();

    TFT_fillScreen(BLACK);

    x1 = y1 = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = 0;
    y2    = h - 1;
    for (x2 = 0; x2 < w; x2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = 0;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);
    x2    = w - 1;
    for (y2 = 0; y2 < h; y2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);

    TFT_fillScreen(BLACK);

    x1    = w - 1;
    y1    = h - 1;
    y2    = 0;
    for (x2 = 0; x2 < w; x2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);
    x2    = 0;
    for (y2 = 0; y2 < h; y2 += 6)
    	TFT_drawLine(x1, y1, x2, y2, color);

}

void TFT_testFastLines(uint16_t color1, uint16_t color2)
{
    int           x, y, w = TFT_width(), h = TFT_height();

    TFT_fillScreen(BLACK);
    for (y = 0; y < h; y += 5)
    	TFT_drawFastHLine(0, y, w, color1);
    for (x = 0; x < w; x += 5)
    	TFT_drawFastVLine(x, 0, h, color2);
}

void TFT_testRects(uint16_t color) {
    int           n, i, i2,
                  cx = TFT_width()  / 2,
                  cy = TFT_height() / 2;

    TFT_fillScreen(BLACK);
    n     = min(TFT_width(), TFT_height());
    for (i = 2; i < n; i += 6) {
        i2 = i / 2;
        TFT_drawRect(cx - i2, cy - i2, i, i, color);
    }

}

void TFT_testFilledRects(uint16_t color1, uint16_t color2)
{
    int           n, i, i2,
                  cx = TFT_width()  / 2 - 1,
                  cy = TFT_height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n = min(TFT_width(), TFT_height());
    for (i = n; i > 0; i -= 6) {
        i2    = i / 2;

        TFT_fillRect(cx - i2, cy - i2, i, i, color1);

        TFT_drawRect(cx - i2, cy - i2, i, i, color2);
    }
}

void TFT_testFilledCircles(uint8_t radius, uint16_t color)
{
    int x, y, w = TFT_width(), h = TFT_height(), r2 = radius * 2;

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
                        w = TFT_width()  + radius,
                        h = TFT_height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    for (x = 0; x < w; x += r2) {
        for (y = 0; y < h; y += r2) {
            TFT_drawCircle(x, y, radius, color);
        }
    }

}

void TFT_testTriangles(void) {
    int           n, i, cx = TFT_width()  / 2 - 1,
                        cy = TFT_height() / 2 - 1;

    TFT_fillScreen(BLACK);
    n     = min(cx, cy);
    for (i = 0; i < n; i += 5) {
        TFT_drawTriangle(
            cx    , cy - i, // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            TFT_color565(0, 0, i));
    }

}

void TFT_testFilledTriangles(void) {
    int           i, cx = TFT_width()  / 2 - 1,
                     cy = TFT_height() / 2 - 1;

    TFT_fillScreen(BLACK);
    for (i = min(cx, cy); i > 10; i -= 5) {
        TFT_fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, TFT_color565(0, i, i));
        TFT_drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, TFT_color565(i, i, 0));
    }
}

void TFT_testRoundRects(void) {
    int           w, i, i2, red, step,
                  cx = TFT_width()  / 2 - 1,
                  cy = TFT_height() / 2 - 1;

    TFT_fillScreen(BLACK);
    w     = min(TFT_width(), TFT_height());
    red = 0;
    step = (256 * 6) / w;
    for (i = 0; i < w; i += 6) {
        i2 = i / 2;
        red += step;
        TFT_drawRoundRect(cx - i2, cy - i2, i, i, i / 8, TFT_color565(red, 0, 0));
    }

}

void TFT_testFilledRoundRects(void) {
    int           i, i2, green, step,
                  cx = TFT_width()  / 2 - 1,
                  cy = TFT_height() / 2 - 1;

    TFT_fillScreen(BLACK);
    green = 256;
    step = (256 * 6) / min(TFT_width(), TFT_height());
    for (i = min(TFT_width(), TFT_height()); i > 20; i -= 6) {
        i2 = i / 2;
        green -= step;
        TFT_fillRoundRect(cx - i2, cy - i2, i, i, i / 8, TFT_color565(0, green, 0));
    }

}



