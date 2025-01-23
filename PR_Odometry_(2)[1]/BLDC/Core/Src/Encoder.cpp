/*
 * Encoder.cpp
 *
 *  Created on: Dec 10, 2023
 *      Author: HOME
 */


#include "main.h"
#include "Encoder.h"

void Encoder::encoder_Init(TIM_HandleTypeDef *timer, uint32_t CHANNEL) {

	HAL_TIM_Encoder_Start_IT(timer, CHANNEL);
}

void Encoder::encoder_update(TIM_HandleTypeDef *timer, uint32_t CHANNEL) {
	count = (int16_t) __HAL_TIM_GET_COUNTER(timer);
	if (count > ccr) {
		__HAL_TIM_SET_COUNTER(timer, 0);
		rotation++;
	} else if (count < -ccr) {
		__HAL_TIM_SET_COUNTER(timer, 0);
		rotation--;
	}
}

long long int Encoder::encoder_read() {
	ticks = ccr * rotation + count;
	return ticks;
}

