/*
 * motor_tb6612.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "motor_tb6612.h"

#include <stdlib.h>

void motorTb_Init(struct motorTb* mTb, TIM_HandleTypeDef* htim,
__IO uint32_t* pwmOutputReg, uint32_t channel, GPIO_TypeDef* dirForward_Port,
		uint16_t dirForward_Pin, GPIO_TypeDef* dirBackward_Port,
		uint16_t dirBackward_Pin) {
	mTb->htim = htim;
	mTb->channel = channel;
	mTb->pwmOutputReg = pwmOutputReg;
	mTb->dirForward_Pin = dirForward_Pin;
	mTb->dirForward_Port = dirForward_Port;
	mTb->dirBackward_Pin = dirBackward_Pin;
	mTb->dirBackward_Port = dirBackward_Port;


	HAL_TIM_PWM_Start(mTb->htim, mTb->channel);

	motorTb_Write(mTb, 0);
}

void motorTb_Write(struct motorTb* mTb, int16_t power) {
	if (power == 0) {
		HAL_GPIO_WritePin(mTb->dirForward_Port, mTb->dirForward_Pin, 0);
		HAL_GPIO_WritePin(mTb->dirBackward_Port, mTb->dirBackward_Pin, 0);
	} else if (power > 0) {
		HAL_GPIO_WritePin(mTb->dirForward_Port, mTb->dirForward_Pin, 1);
		HAL_GPIO_WritePin(mTb->dirBackward_Port, mTb->dirBackward_Pin, 0);
	} else if (power < 0) {
		HAL_GPIO_WritePin(mTb->dirForward_Port, mTb->dirForward_Pin, 0);
		HAL_GPIO_WritePin(mTb->dirBackward_Port, mTb->dirBackward_Pin, 1);
	}

	*mTb->pwmOutputReg = abs(power);

}
