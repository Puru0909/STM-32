#include "TriBase.h"
#include "math.h"

#define PI 3.141592

// Constructor: Initialize with references to existing MOTOR objects
TriBaseKinematics::TriBaseKinematics(MOTOR& motor1, MOTOR& motor2, MOTOR& motor3): motor1(motor1), motor2(motor2), motor3(motor3) {

}

void TriBaseKinematics::set_base_speeds(float motor1_base_speed, float motor2_base_speed, float motor3_base_speed){
	this->motor1_base_speed = motor1_base_speed;
	this->motor2_base_speed = motor2_base_speed;
	this->motor3_base_speed = motor3_base_speed;
}

void TriBaseKinematics::update_motor_speeds(){
	motor1.set_speed(speed1);
	motor2.set_speed(speed1);
	motor3.set_speed(speed1);
}

void TriBaseKinematics::go_at_angle(float current_angle, float change_in_x, float change_in_y, float change_in_angle) {
	if (current_angle == 0 && change_in_x == 0 && change_in_y == 0 && change_in_angle == 0){
		this->brake();
		return;
	}
	float s1 = (1/0.076235) * ((-sin(current_angle)*cos(current_angle)*change_in_x) + (cos(current_angle)*cos(current_angle)*change_in_y) + (0.28145825622994*change_in_angle));
	float s2 = (1/0.076235) * ((-sin(current_angle+(alpha2))*cos(current_angle)*change_in_x) + (cos(current_angle+(alpha2))*cos(current_angle)*change_in_y) + (0.28145825622994*change_in_angle));
	float s3 = (1/0.076235) * ((-sin(current_angle+(alpha3))*cos(current_angle)*change_in_x) + (cos(current_angle+(alpha3))*cos(current_angle)*change_in_y) + (0.28145825622994*change_in_angle));
	float max = ((abs(s1)>abs(s2)) && (abs(s1)>abs(s3)))? abs(s1) : (abs(s2)>abs(s3)? abs(s2) : abs(s3));
	s1 = s1*100/max;
	s2 = s2*100/max;
	s3 = s3*100/max;
	this->speed1 = s1;
	this->speed2 = s2;
	this->speed3 = s3;
	this->update_motor_speeds();
}

void TriBaseKinematics::brake() {
	speed1 = 0;
	speed2 = 0;
	speed3 = 0;
	this->update_motor_speeds();
}
