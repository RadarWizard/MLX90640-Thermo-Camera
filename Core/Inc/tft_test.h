/*
 * tft_test.h
 *
 *  Created on: 5 Jul 2022
 *      Author: Peter
 */

#ifndef INC_TFT_TEST_H_
#define INC_TFT_TEST_H_

void TFT_testFastLines(uint16_t color1, uint16_t color2);
void TFT_testLines(uint16_t color);
void TFT_testFillScreen(void);
void TFT_testRects(uint16_t color);
void TFT_testFilledRects(uint16_t color1, uint16_t color2);
void TFT_testFilledCircles(uint8_t radius, uint16_t color) ;
void TFT_testCircles(uint8_t radius, uint16_t color);
void TFT_testFilledRoundRects(void);
void TFT_testRoundRects(void);
void TFT_testFilledTriangles(void);
void TFT_testTriangles(void);



#endif /* INC_TFT_TEST_H_ */
