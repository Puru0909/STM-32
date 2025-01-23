

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
	motor_2->anti_clockwise(pwm_2);
	motor_3->clockwise(pwm_3);
}
void tribase_kinematics::backward(uint8_t pwm_2, uint8_t pwm_3) {
  motor_1->brake();
  motor_2->clockwise(pwm_2);
  motor_3->anti_clockwise(pwm_3);
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


    float r = 0.03;
    float d = 0.18;


    float speedA = (1 / r) * ((-d * omega_bz) + (v_bx ));
    float speedB = (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx ) + ((-1.73) * v_by ));
    float speedC =  (1 / r) * ((-d * omega_bz) + ((-0.5) * v_bx) + (1.73 * v_by ));


   motor_1->set_speed(speedA);  // Set speed instead of assigning directly
   motor_2->set_speed(speedB);
   motor_3->set_speed(speedC);

}

void tribase_kinematics::brake() {
  motor_1->brake();
  motor_2->brake();
  motor_3->brake();

}




