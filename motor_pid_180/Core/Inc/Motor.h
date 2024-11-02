/*
 * Motor.h
 *
 *  Created on: Oct 6, 2024
 *      Author: PURUSHOTTAM
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#define TARGET_POSITION 180  // Replace with actual encoder counts for 180 degrees
#define TOLERANCE 5          // Tolerance range around target position

#include "main.h"

class MOTOR{
protected:
	GPIO_TypeDef *PORTA, *PORTB;
	uint16_t Pin1, Pin2;
	TIM_HandleTypeDef * timer;
	uint32_t CHANNEL;


public:
	MOTOR(GPIO_TypeDef * PORTA, uint16_t Pin1, GPIO_TypeDef * PORTB, uint16_t Pin2);
	void motor_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
	void clockwise(int pwm);
	void anti_clockwise(int pwm);
	void brake();

};



#endif /* INC_MOTOR_H_ */
