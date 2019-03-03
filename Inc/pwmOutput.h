/*
 * pwmOutput.h
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#ifndef PWMOUTPUT_H_
#define PWMOUTPUT_H_

#include "stm32f1xx_hal.h"

struct pwmOutput {
	TIM_HandleTypeDef* htim;
	uint32_t channel;
	__IO uint32_t* pwmOutputReg;
};

void pwmOut_Init(struct pwmOutput* pwmOut, TIM_HandleTypeDef* htim,
		uint32_t channel,
		__IO uint32_t* pwmOutputReg);

void pwmOut_WriteMotor(struct pwmOutput* pwmOut, uint16_t value);

#endif /* PWMOUTPUT_H_ */
