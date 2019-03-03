/*
 * utility.c
 *
 *  Created on: Apr 21, 2018
 *      Author: kacper
 */
#include "utility.h"

#include "stm32f1xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;

int _write(int file, char* ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, len, 500);
	return len;
}
