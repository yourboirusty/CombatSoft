/*
 * SanwaSynchronaizedLink.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "SanwaSynchronaizedLink.h"

#include "stm32f1xx_hal.h"

uint8_t SSL_checkSum(uint8_t* buffer);

#define SSL_RX_BUFFER_LEN	20

struct SanwaSynchronaizedLink {
	UART_HandleTypeDef* huart;
} ssl;

uint8_t rx_buffer[SSL_RX_BUFFER_LEN];
uint8_t* Processed;
uint8_t buf[64];
static uint16_t i;

void SSL_Init(UART_HandleTypeDef *huart) {
	ssl.huart = huart;
	HAL_UART_Receive_DMA(ssl.huart, rx_buffer, SSL_RX_BUFFER_LEN);
}

void SSL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// skip function if huart does not match
	if (huart != ssl.huart) {
		return;
	}

	Processed = rx_buffer;
	while (!SSL_checkSum(Processed)) {
		Processed = Processed + 1;
	}

	control_data.status = Processed[0];
	control_data.ch1 = (Processed[1] << 8) + Processed[2];
	control_data.ch2 = (Processed[3] << 8) + Processed[4];
	control_data.ch3 = (Processed[5] << 8) + Processed[6];
	control_data.ch4 = (Processed[7] << 8) + Processed[8];

	HAL_UART_Receive_DMA(ssl.huart, rx_buffer, SSL_RX_BUFFER_LEN);
}

uint8_t SSL_checkSum(uint8_t* buffer) {
	uint8_t controlSum = buffer[9];
	uint8_t controlSumBuffer = 0;
	for (i = 0; i < 9; i++) {
		controlSumBuffer += buffer[i];
	}
	return controlSumBuffer == controlSum;
}

