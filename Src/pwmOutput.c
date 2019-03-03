/*
 * pwmOutput.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "pwmOutput.h"

void pwmOut_Init(struct pwmOutput* pwmOut, TIM_HandleTypeDef* htim,
		uint32_t channel,
		__IO uint32_t* pwmOutputReg) {
	pwmOut->htim = htim;
	pwmOut->channel = channel;
	pwmOut->pwmOutputReg = pwmOutputReg;
	pwmOut_WriteMotor(pwmOut, 0);
}

void pwmOut_WriteMotor(struct pwmOutput* pwmOut, uint16_t value) {
	*pwmOut->pwmOutputReg = 100;
}
