/*
 * Encoder.h
 *
 *  Created on: Dec 10, 2023
 *      Author: HOME
 */

#ifndef SRC_ENCODER_H_
#define SRC_ENCODER_H_

#include "main.h"

class Encoder {
public:
	int16_t count = 0;
	int16_t ccr;
	int16_t rotation = 0;
	int16_t diameter;
	long long int ticks = 0;

	void encoder_Init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
	void encoder_update(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
	long long int encoder_read();
	float distance();

};

#endif /* SRC_ENCODER_H_ */
