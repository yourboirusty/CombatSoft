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

void userMain() {
	SSL_Init(&huart1);

	while (1) {
		HAL_Delay(100);

		if (control_data.status == 3 || control_data.status == 0)
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		else {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
			if (control_data.ch1 < 800)
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
			else
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
			if (control_data.ch2 < 700)
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			else
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
		}

	}
}
