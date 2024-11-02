/*
 * Tribase.c
 *
 *  Created on: Oct 26, 2024
 *      Author: PURUSHOTTAM
 */

#include "Tribase.h"
#include "Motor.h"
#include "main.h"



tribase_kinematics::tribase_kinematics(MOTOR* motor_1, MOTOR* motor_2, MOTOR* motor_3){
	this->motor_1 = motor_1;
	this->motor_2 = motor_2;
	this->motor_3 = motor_3;
}
MOTOR motor_1();
MOTOR motor_2();
MOTOR motor_3();


//void MOTOR::motor_init(TIM_HandleTypeDef * timer, uint32_t CHANNEL){
//	this->timer = timer;
//	this->CHANNEL = CHANNEL;
//	HAL_TIM_PWM_Start(timer, CHANNEL);
//}

void tribase_kinematics::forward(uint8_t pwm_2, uint8_t pwm_3){
	motor_1->brake();
	motor_2->clockwise(pwm_2);
	motor_3->anti_clockwise(pwm_3);
}
void tribase_kinematics::backward(uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->brake();
  motor_2->anti_clockwise(pwm_2);
  motor_3->clockwise(pwm_3);
}

void tribase_kinematics::left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->clockwise(pwm_1);
  motor_2->anti_clockwise(pwm_2);
  motor_3->anti_clockwise(pwm_3);
}

void tribase_kinematics::right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->anti_clockwise(pwm_1);
  motor_2->clockwise(pwm_2);
  motor_3->clockwise(pwm_3);
}
//void tribase_kinematics::forward_left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->brake();
//  motor_2->clockwise(pwm_2);
//  motor_3->anti_clockwise(pwm_3);
//}
//
//void tribase_kinematics::forward_right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->anti_clockwise(pwm_1);
//  motor_2->clockwise(pwm_2);
//  motor_3->brake();
//}
//void tribase_kinematics::right_forward(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->anti_clockwise(pwm_1);
//  motor_2->clockwise(pwm_2);
//  motor_3->clockwise(pwm_3);
//}
//void tribase_kinematics::backward_left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->clockwise(pwm_1);
//  motor_2->anti_clockwise(pwm_2);
//  motor_3->brake();
//}
//
//void tribase_kinematics::backward_right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->brake();
//  motor_2->anti_clockwise(pwm_2);
//  motor_3->clockwise(pwm_3);
//}
//void tribase_kinematics::right_bakward(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
//  motor_1->anti_clockwise(pwm_1);
//  motor_2->anti_clockwise(pwm_2);
//  motor_3->clockwise(pwm_3);
//}
void tribase_kinematics::anti_clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->clockwise(pwm_1);
  motor_2->clockwise(pwm_2);
  motor_3->clockwise(pwm_3);
}

void tribase_kinematics::clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->anti_clockwise(pwm_1);
  motor_2->anti_clockwise(pwm_2);
  motor_3->anti_clockwise(pwm_3);

}
void tribase_kinematics::brake() {
  motor_1->brake();
  motor_2->brake();
  motor_3->brake();

}



