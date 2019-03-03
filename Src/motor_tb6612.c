/*
 * motor_tb6612.c
 *
 *  Created on: Mar 3, 2019
 *      Author: kacper
 */

#include "motor_tb6612.h"

void motorTb_Init(struct motorTb* mTb, TIM_HandleTypeDef* htim,
		uint32_t channel, GPIO_TypeDef* dirForward_Port,
		uint16_t dirForward_Pin, GPIO_TypeDef* dirBackward_Port,
		uint16_t dirBackward_Pin) {
	mTb->htim = htim;
	mTb->channel = channel;
	mTb->dirForward_Pin = dirForward_Pin;
	mTb->dirForward_Port = dirForward_Port;
	mTb->dirBackward_Pin = dirBackward_Pin;
	mTb->dirBackward_Port = dirBackward_Port;

	motorTb_Write(mTb, 0);

	HAL_TIM_PWM_Start(mTb->htim, mTb->channel);
}

void motorTb_Write(struct motorTb* mTb, int16_t power) {

}
