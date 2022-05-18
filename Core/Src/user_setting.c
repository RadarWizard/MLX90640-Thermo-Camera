/*
 * user_setting.c
 *
 *  Created on: Apr 30, 2022
 *      Author: Peter
 */

#include "main.h"
/****************** delay in microseconds ***********************/
extern TIM_HandleTypeDef htim1;
void delay (uint32_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}

/*PRW The following two functions are very inefficient. They call the library function HAL_GPIO_Init
 * which is a very long function when we only want to reconfigure the the pins as inputs and outputs*/
void setReadDir (void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	/*Configure GPIO pins : LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin LCD_D4_Pin */
	GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_D1_Pin */
	GPIO_InitStruct.Pin = LCD_D1_Pin;
	HAL_GPIO_Init(LCD_D1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D7_Pin LCD_D0_Pin LCD_D2_Pin */
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D0_Pin|LCD_D2_Pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void setWriteDir (void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	/*Configure GPIO pins : LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin LCD_D4_Pin */
	GPIO_InitStruct.Pin = LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_D1_Pin */
	GPIO_InitStruct.Pin = LCD_D1_Pin;
	HAL_GPIO_Init(LCD_D1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D7_Pin LCD_D0_Pin LCD_D2_Pin */
	GPIO_InitStruct.Pin = LCD_D7_Pin|LCD_D0_Pin|LCD_D2_Pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

