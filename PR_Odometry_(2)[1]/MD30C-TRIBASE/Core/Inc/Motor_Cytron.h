/*
 * Motor_Cytron.h
 *
 *  Created on: Nov 29, 2024
 *      Author: Dell
 */

#ifndef INC_MOTOR_CYTRON_H_
#define INC_MOTOR_CYTRON_H_
#define DUMMY_BIT 128
#define HEADER 85

#include "main.h"
class Motor {
public:
	Motor(GPIO_TypeDef* PORT_1, uint16_t Pin1, TIM_HandleTypeDef *htim, uint32_t Channel);
	void init();
	void clockwise(uint8_t pwm);
	void anti_clockwise(uint8_t pwm);
	void brake();
	void setSpeed(int speed);

private:

	GPIO_TypeDef* PORT_1;
	uint16_t Pin1;
	TIM_HandleTypeDef *htim;
	uint32_t Channel;


};




#endif /* INC_MOTOR_COMMON_H_ */

