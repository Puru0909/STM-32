/*
 * TriBaseKinematics.h
 *
 *  Created on: Oct 26, 2024
 *      Author: PURUSHOTTAM
 */

#ifndef INC_TRIBASE_H_
#define INC_TRIBASE_H_

#include "main.h"
#include "Motor.h"


class tribase_kinematics {
protected:
	MOTOR* motor_1;
	MOTOR* motor_2;
	MOTOR* motor_3;

public:

    tribase_kinematics(MOTOR* motor_1, MOTOR* motor_2, MOTOR* motor_3);
    void motor_init(TIM_HandleTypeDef *timer, uint32_t CHANNEL);
    void forward(uint8_t pwm_2, uint8_t pwm_3);
    void backward(uint8_t pwm_2, uint8_t pwm_3);
    void right(uint8_t pwm_1,uint8_t pwm_2,uint8_t pwm_3);
    void left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
//    void forward_left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
//    void forward_right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
//    void backward_left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
//    void backward_right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void anti_clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void brake();
};

#endif /* INC_TRIBASEKINEMATICS_H_ */
