/*
 * utility.c
 *
 *  Created on: Apr 21, 2018
 *      Author: kacper
 */
#include "utility.h"

#include "stm32f1xx_hal.h"
#include <stdio.h>

//extern UART_HandleTypeDef huart2;

int _write(int file, char* ptr, int len) {
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) ptr, len);
	return len;
}
