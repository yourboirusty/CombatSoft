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

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	SSL_UART_RxCpltCallback(huart);
}

int8_t parse_raw_stick(int16_t input) {
	input = (input - 1024) * 1.57;

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

	}
}
