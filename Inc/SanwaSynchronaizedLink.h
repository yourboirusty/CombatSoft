/*
 * SanwaSynchronaizedLink.h
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#ifndef SANWASYNCHRONAIZEDLINK_H_
#define SANWASYNCHRONAIZEDLINK_H_

#include "stm32f1xx_hal.h"

struct controls {
	uint8_t status; // 01 - ok, 02 - retransmit, 03 - timed out
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
} control_data;

void SSL_Init(UART_HandleTypeDef *huart);

void SSL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SANWASYNCHRONAIZEDLINK_H_ */
