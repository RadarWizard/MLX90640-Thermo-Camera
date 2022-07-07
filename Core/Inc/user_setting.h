/*
 * user_setting.h
 *
 *  Created on: 02-Jul-2019
 *      Author: poe
 */

#ifndef USER_SETTING_H_
#define USER_SETTING_H_

#include "main.h"

/*Touchscreen defines*/
#define LCD_TS_YP_Pin GPIO_PIN_4
#define LCD_TS_YP_GPIO_Port GPIOA
#define LCD_TS_XM_Pin GPIO_PIN_0
#define LCD_TS_XM_GPIO_Port GPIOB
#define LCD_TS_YM_Pin GPIO_PIN_9
#define LCD_TS_YM_GPIO_Port GPIOA
#define LCD_TS_XP_Pin GPIO_PIN_7
#define LCD_TS_XP_GPIO_Port GPIOC
#define LCD_TS_YP_ADC_CHAN ADC_CHANNEL_9
#define LCD_TS_XM_ADC_CHAN ADC_CHANNEL_15

#define RD_PORT LCD_RD_GPIO_Port
#define RD_PIN  LCD_RD_Pin
#define WR_PORT LCD_WR_GPIO_Port
#define WR_PIN  LCD_WR_Pin
#define CD_PORT LCD_RS_GPIO_Port          // RS PORT
#define CD_PIN  LCD_RS_Pin     // RS PIN
#define CS_PORT LCD_CS_GPIO_Port
#define CS_PIN  LCD_CS_Pin
#define RESET_PORT LCD_RST_GPIO_Port
#define RESET_PIN  LCD_RST_Pin

#define D0_PORT LCD_D0_GPIO_Port
#define D0_PIN LCD_D0_Pin
#define D1_PORT LCD_D1_GPIO_Port
#define D1_PIN LCD_D1_Pin
#define D2_PORT LCD_D2_GPIO_Port
#define D2_PIN LCD_D2_Pin
#define D3_PORT LCD_D3_GPIO_Port
#define D3_PIN LCD_D3_Pin
#define D4_PORT LCD_D4_GPIO_Port
#define D4_PIN LCD_D4_Pin
#define D5_PORT LCD_D5_GPIO_Port
#define D5_PIN LCD_D5_Pin
#define D6_PORT LCD_D6_GPIO_Port
#define D6_PIN LCD_D6_Pin
#define D7_PORT LCD_D7_GPIO_Port
#define D7_PIN LCD_D7_Pin

/*ILI9488 display is a 320 by 480 pixel display*/
#define  WIDTH    ((uint16_t)320)
#define  HEIGHT   ((uint16_t)480)

extern void delay (uint32_t time);
extern void setReadDir (void);
extern void setWriteDir (void);


// configure macros for the data pins.

/* First of all clear all the LCD_DATA pins i.e. LCD_D0 to LCD_D7
 * We do that by writing the HIGHER bits in BSRR Register
 *
 * For example :- To clear Pins B3, B4 , B8, B9, we have to write GPIOB->BSRR = 0b0000001100011000 <<16
 *
 *
 *
 * To write the data to the respective Pins, we have to write the lower bits of BSRR :-
 *
 * For example say the PIN LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
 *
 * GPIOB->BSRR = (data & (1<<4)) << 3.  Here first select 4th bit of data (LCD_D4), and than again shift left by 3 (Total 4+3 =7 i.e. PB7)
 *
 * GPIOB->BSRR = (data & (1<<6)) >> 4.  Here first select 6th bit of data (LCD_D6), and than again shift Right by 4 (Total 6-4 =2 i.e. PB2)
 *
 *
 */
  #define write_8(d) { \
   GPIOA->BSRR = 0b0000011100000000 << 16; \
   GPIOB->BSRR = 0b0000010000111000 << 16; \
   GPIOC->BSRR = 0b0000000010000000 << 16; \
   GPIOA->BSRR = (((d) & (1<<0)) << 9) \
               | (((d) & (1<<2)) << 8) \
			   | (((d) & (1<<7)) << 1); \
   GPIOB->BSRR = (((d) & (1<<3)) << 0) \
               | (((d) & (1<<4)) << 1) \
			   | (((d) & (1<<5)) >> 1) \
			   | (((d) & (1<<6)) << 4); \
   GPIOC->BSRR = (((d) & (1<<1)) << 6); \
    }


  /* To read the data from the Pins, we have to read the IDR Register
   *
   * Take the same example say LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
   *
   * To read data we have to do the following
   *
   * GPIOB->IDR & (1<<7) >> 3. First read the PIN (1<<7 means we are reading PB7) than shift it to the position, where it is connected to
   * and in this example, that would be 4 (LCD_D4). (i.e. 7-3=4)
   *
   * GPIOB->IDR & (1<<2) << 4. First read the PIN (1<<2 means we are reading PB2) than shift it to the position, where it is connected to
   * and in this case, that would be 6 (LCD_D6). (i.e. 2+4= 6). Shifting in the same direction
   *
   */
  #define read_8() (          (((GPIOA->IDR & (1<<9)) >> 9) \
                           | ((GPIOC->IDR & (1<<7)) >> 6) \
                           | ((GPIOA->IDR & (1<<10)) >> 8) \
                           | ((GPIOB->IDR & (1<<3)) >> 0) \
                           | ((GPIOB->IDR & (1<<5)) >> 1) \
                           | ((GPIOB->IDR & (1<<4)) << 1) \
                           | ((GPIOB->IDR & (1<<10)) >> 4) \
                           | ((GPIOA->IDR & (1<<8)) >> 1)))


/********************* For 180 MHz *****************************/
//#define WRITE_DELAY { WR_ACTIVE8; }
//#define READ_DELAY  { RD_ACTIVE16;}


/************************** For 72 MHZ ****************************/
#define WRITE_DELAY { }
#define READ_DELAY  { RD_ACTIVE;  }


/************************** For 100 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE2; }
//#define READ_DELAY  { RD_ACTIVE4; }


/************************** For 216 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE8; WR_ACTIVE8; } //216MHz
//#define IDLE_DELAY  { WR_IDLE4;WR_IDLE4; }
//#define READ_DELAY  { RD_ACTIVE16;RD_ACTIVE16;RD_ACTIVE16;}


/************************** For 48 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { }


/*****************************  DEFINES FOR DIFFERENT TFTs   ****************************************************/

#define SUPPORT_9488_555          //costs +230 bytes, 0.03s / 0.19s




#endif /* USER_SETTING_H_ */
