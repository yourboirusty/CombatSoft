/*
 * SanwaSynchronaizedLink.h
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#ifndef SANWASYNCHRONAIZEDLINK_H_
#define SANWASYNCHRONAIZEDLINK_H_

#include "stm32f1xx_hal.h"

void SSL_Init(UART_HandleTypeDef *huart);

void SSL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* SANWASYNCHRONAIZEDLINK_H_ */
