/*
 * SanwaSynchronaizedLink.h
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#ifndef SANWASYNCHRONAIZEDLINK_H_
#define SANWASYNCHRONAIZEDLINK_H_

#include "stm32f1xx_hal.h"

enum SSL_Status{
	SSL_STATUS_OK = 0x01,
	SSL_STATUS_RETRANSMIT = 0x02,
	SSL_STATUS_TIMED_OUT = 0x03
};

struct controls {
	enum SSL_Status status;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t valueCh[4];
	uint8_t dataReady:1;
	uint8_t dataAck:1;
} control_data;

void SSL_Init(UART_HandleTypeDef *huart);

void SSL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SANWASYNCHRONAIZEDLINK_H_ */
