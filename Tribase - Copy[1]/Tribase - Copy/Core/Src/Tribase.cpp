/*
 * Tribase.cpp
 *
 *  Created on: Nov 11, 2024
 *      Author: PURUSHOTTAM
 */
/*
 * Tribase.c
 *
 *  Created on: Oct 26, 2024
 *      Author: PURUSHOTTAM
 */

#include "Tribase.h"
#include "Motor.h"




tribase_kinematics::tribase_kinematics(MOTOR* motor_1, MOTOR* motor_2, MOTOR* motor_3){
	this->motor_1 = motor_1;
	this->motor_2 = motor_2;
	this->motor_3 = motor_3;
}
MOTOR motor_1();
MOTOR motor_2();
MOTOR motor_3();


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


void tribase_kinematics::IK(float omega_bz, float v_bx, float v_by) {


    float r = 0.063;
    float d = 0.24;

    float speedA = (1 / r) * ((-d * omega_bz) + (v_bx ));
    float speedB = (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx ) + ((-0.866) * v_by ));
    float speedC =  (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx) + (0.866 * v_by ));


    //for 45 degree quad base
   // v_w = (1 / r) * [ -d * ω_bz + cos(β) * v_bx + sin(β) * v_by ]

//    float speedA = (1 / r) * ((-d * omega_bz) + (0.707 * v_bx) + (0.707 * v_by));
//    float speedB = (1 / r) * ((-d * omega_bz) + (-0.707 * v_bx) + (0.707 * v_by));
//    float speedC = (1 / r) * ((-d * omega_bz) + (-0.707 * v_bx) + (-0.707 * v_by));
//    float speedD = (1 / r) * ((-d * omega_bz) + (0.707 * v_bx) + (-0.707 * v_by));








   motor_1->setSpeed(speedA);  // Set speed instead of assigning directly
   motor_2->setSpeed(speedB);
   motor_3->setSpeed(speedC);

}





//    float tribase_kinematics::getWheelVelocity1()  {
//        return motor_1;
//
//
//    }
//    float tribase_kinematics::getWheelVelocity2()  {
//        return motor_2;
//    }
//    float tribase_kinematics::getWheelVelocity3()  {
//        return motor_3;
//    }



void tribase_kinematics::brake() {
  motor_1->brake();
  motor_2->brake();
  motor_3->brake();

}




