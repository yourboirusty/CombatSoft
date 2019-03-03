/*
 * motor_tb6612.h
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#ifndef MOTOR_TB6612_H_
#define MOTOR_TB6612_H_

#include "stm32f1xx_hal.h"

struct motorTb {
	TIM_HandleTypeDef* htim;
	uint32_t channel;
	__IO uint32_t* pwmOutputReg;
	GPIO_TypeDef* dirForward_Port;
	uint16_t dirForward_Pin;
	GPIO_TypeDef* dirBackward_Port;
	uint16_t dirBackward_Pin;
};

void motorTb_Init(struct motorTb* mTb, TIM_HandleTypeDef* htim,
__IO uint32_t* pwmOutputReg, uint32_t channel, GPIO_TypeDef* dirForward_Port,
		uint16_t dirForward_Pin, GPIO_TypeDef* dirBackward_Port,
		uint16_t dirBackward_Pin);

void motorTb_Write(struct motorTb* mTb, int16_t power);

#endif /* MOTOR_TB6612_H_ */
