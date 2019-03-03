/*
 * userMain.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "userMain.h"

#include "stm32f1xx_hal.h"

#include "SanwaSynchronaizedLink.h"



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	SSL_UART_RxCpltCallback(huart);
}

void userMain() {

}
