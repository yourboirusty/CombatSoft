/*
 * userMain.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "userMain.h"

#include "stm32f1xx_hal.h"

#include "utility.h"

#include "main.h"

#include "SanwaSynchronaizedLink.h"
#include "motor_tb6612.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim3;

struct motorTb motorLeft;
struct motorTb motorRight;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	SSL_UART_RxCpltCallback(huart);
}

int16_t parse_raw_stick(int16_t input) {
	input = (double) 1.57 * (input - 1024);

	if (input > 1000) {
		input = 1000;
	}

	if (input < -1000) {
		input = -1000;
	}

	return input;
}

void userMain() {
	SSL_Init(&huart1);
//
	motorTb_Init(&motorLeft, &htim3, &TIM3->CCR1, TIM_CHANNEL_1, ENG_L_DIRA_GPIO_Port, ENG_L_DIRA_Pin, ENG_L_DIRB_GPIO_Port, ENG_L_DIRB_Pin);
//	motorTb_Init(&motorRight, &htim3, &TIM3->CCR2, TIM_CHANNEL_2, ENG_R_DIRA_GPIO_Port, ENG_R_DIRA_Pin, ENG_R_DIRB_GPIO_Port, ENG_R_DIRB_Pin);

	int16_t kolo;

	while (1) {
		HAL_Delay(100);

		// does radio work?
		if (control_data.status == SSL_STATUS_OK) {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
		} else {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		}

		kolo = parse_raw_stick(control_data.valueCh[0]);

		if (kolo > 500)
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		else
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);

		if (kolo < -500)
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		else
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);


		motorTb_Write(&motorLeft, kolo);
	}
}
