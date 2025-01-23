/*
 * Tribase.h
 *
 *  Created on: Nov 29, 2024
 *      Author: Dell
 */

#ifndef INC_TRIBASE_H_
#define INC_TRIBASE_H_

#include "main.h"
#include "CYTRON.h"
#include "Motor_Cytron.h"

class tribase_kinematics{
protected:
	Motor* motor_1;
	Cytron* motor_2;
	Cytron* motor_3;

public:
	tribase_kinematics(Motor* motor_1, Cytron* motor_2, Cytron* motor_3);
	void forward(uint8_t pwm_1, uint8_t pwm_2);
	void backward(uint8_t pwm_1, uint8_t pwm_2);
	void left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
	void right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
	void clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
	void anti_clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
	void IK(float omega_bz, float v_bx, float v_by);
	void brake();
};


#endif /* INC_TRIBASE_LIB_H_ */
 /* INC_TRIBASE_H_ */
