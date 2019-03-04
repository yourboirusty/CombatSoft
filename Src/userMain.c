/*
 * userMain.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "userMain.h"

#include "stm32f1xx_hal.h"

#include <stdlib.h>

#include "utility.h"

#include "main.h"

#include "SanwaSynchronaizedLink.h"
#include "motor_tb6612.h"
#include "pwmOutput.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

int16_t gaz;
int16_t kolo;
struct motorTb motorLeft;
struct motorTb motorRight;
struct pwmOutput BLDC;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	SSL_UART_RxCpltCallback(huart);
}

void panic() {
	motorTb_Write(&motorLeft, 0);
	motorTb_Write(&motorRight, 0);
	pwmOut_WriteMotor(&BLDC, 0);
}

int16_t liczenie_kola(int16_t input) {
	int16_t output = (double) 1.57 * (input - 1024);

	if (output > 1000) {
		output = 1000;
	}

	if (output < -1000) {
		output = -1000;
	}

	return output;
}

int16_t liczenie_gazu(int16_t input) {
	//	int16_t output = (double) 1.57 * (input - 1024);
	int16_t output = (double) 1.22 * (input - 776);

	if (output > 1000) {
		output = 1000;
	}

	if (output < -1000) {
		output = -1000;
	}

	return output;
}

void MC_basic(int16_t stickX, int16_t stickY, int16_t* driveL, int16_t* driveR) {
	// temporary variables definition
	int16_t motorL = 0;
	int16_t motorR = 0;

	// algorithm BEGIN

	motorL = stickY + stickX;
	motorR = stickY - stickX;

	if (motorL > 1000)
		motorL = 1000;
	if (motorR > 1000)
		motorR = 1000;

	if (motorL < -1000)
		motorL = -1000;
	if (motorR < -1000)
		motorR = -1000;

//	motorL = motorL *3/5;
//	motorR = motorR *3/5;

// algorithm END

// pass computed values as output
	*driveL = motorL;
	*driveR = motorR;
}

void radioLinkTimeOut(enum SSL_Status status) {
	static uint32_t last = 0;
	uint32_t deltaT;

	if (status == SSL_STATUS_OK) {
		last = HAL_GetTick();
	}

	deltaT = HAL_GetTick() - last;

	if (deltaT > 100) {
		panic();
	}
}

void userMain() {

	int16_t mocL, mocR;

	SSL_Init(&huart1);

	pwmOut_Init(&BLDC, &htim15, TIM_CHANNEL_1, &TIM15->CCR1);
	pwmOut_WriteMotor(&BLDC, 1000);

	//	motorTb_Init(&motorLeft, &htim3, &TIM3->CCR1, TIM_CHANNEL_1,
	//	ENG_L_DIRA_GPIO_Port, ENG_L_DIRA_Pin, ENG_L_DIRB_GPIO_Port,
	//	ENG_L_DIRB_Pin);
	//	motorTb_Init(&motorRight, &htim3, &TIM3->CCR2, TIM_CHANNEL_2,
	//	ENG_R_DIRA_GPIO_Port, ENG_R_DIRA_Pin, ENG_R_DIRB_GPIO_Port,
	//	ENG_R_DIRB_Pin);

	while (1) {
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		HAL_Delay(5);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

		// panic if timed out
		if (control_data.status == SSL_STATUS_TIMED_OUT) {
			panic();
			continue;
		}

		radioLinkTimeOut(control_data.status);

		// led on if radio works
		if (control_data.status == SSL_STATUS_OK) {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
		} else {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
		}

		kolo = liczenie_kola(control_data.valueCh[0]);
		gaz = liczenie_gazu(control_data.valueCh[1]);

		if (kolo > 500)
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		else
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);

		if (kolo < -500)
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		else
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

		MC_basic(kolo, gaz, &mocL, &mocR);

		pwmOut_WriteMotor(&BLDC, abs(kolo));

//		motorTb_Write(&motorLeft, mocL);
//		motorTb_Write(&motorRight, mocR);
	}
}
