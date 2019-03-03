/*
 * SanwaSynchronaizedLink.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "SanwaSynchronaizedLink.h"

#include "stm32f1xx_hal.h"

void SSL_Init(UART_HandleTypeDef *huart) {
//	uartRxMachinery.huart = huart;
//	uartRxMachinery.uartRxLineCallback = NULL;
//	uartRxMachinery_ResetBuffer();
//	HAL_UART_Receive_DMA(huart, &uartRxMachinery.uartRxChar, 1);
}

void SSL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

//	Processed = Received;
//	while (!CheckSum(Processed))
//		Processed = Processed + 1;
//
//	control_data.status = Processed[0];
//	control_data.ch1 = (Processed[1] << 8) + Processed[2];
//	control_data.ch2 = (Processed[3] << 8) + Processed[4];
//	control_data.ch3 = (Processed[5] << 8) + Processed[6];
//	control_data.ch4 = (Processed[7] << 8) + Processed[8];
//
//	HAL_UART_Receive_DMA(&huart1, Received, rec); // Ponowne w��czenie nas�uchiwania
}


