#include "QuadBase.h"
#include "Motor.h"
#include "math.h"

// Constructor: Initialize with references to existing MOTOR objects
QuadBaseKinematics::QuadBaseKinematics() {

}

void QuadBaseKinematics::set_speed_scalers(float motor1_base_speed, float motor2_base_speed, float motor3_base_speed, float motor4_base_speed) {
//	this->motor1_base_speed = motor1_base_speed;
//	this->motor2_base_speed = motor2_base_speed;
//	this->motor3_base_speed = motor3_base_speed;
//	this->motor4_base_speed = motor4_base_speed;
}

void QuadBaseKinematics::calculate_speeds(float current_angle, float change_in_x, float change_in_y, float change_in_angle) {
	if (current_angle == 0 && change_in_x == 0 && change_in_y == 0 && change_in_angle == 0){
		this->brake();
		return;
	}
	float s1 = (-sin(current_angle+alpha1)*cos(current_angle)*change_in_x) + (cos(current_angle+alpha1)*cos(current_angle)*change_in_y) + (0.3125*change_in_angle);
	float s2 = (-sin(current_angle+alpha2)*cos(current_angle)*change_in_x) + (cos(current_angle+alpha2)*cos(current_angle)*change_in_y) + (0.3125*change_in_angle);
	float s3 = (-sin(current_angle+alpha3)*cos(current_angle)*change_in_x) + (cos(current_angle+alpha3)*cos(current_angle)*change_in_y) + (0.3125*change_in_angle);
	float s4 = (-sin(current_angle+alpha4)*cos(current_angle)*change_in_x) + (cos(current_angle+alpha4)*cos(current_angle)*change_in_y) + (0.3125*change_in_angle);
	float max = 1;
	s1 *= motor1_base_speed/max;
	s2 *= motor2_base_speed/max;
	s3 *= motor3_base_speed/max;
	s4 *= motor4_base_speed/max;
	this->speed1 = s1;
	this->speed2 = s2;
	this->speed3 = s3;
	this->speed4 = s4;
}

void QuadBaseKinematics::brake() {
	this->speed1 = 0;
	this->speed2 = 0;
	this->speed3 = 0;
	this->speed4 = 0;
}
