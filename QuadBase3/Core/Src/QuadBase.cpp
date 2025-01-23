#include "QuadBase.h"
#include "Motor.h"
#include "math.h"

// Constructor: Initialize with references to existing MOTOR objects
QuadBaseKinematics::QuadBaseKinematics() {

}

void QuadBaseKinematics::set_speed_scalers(float motor1_speed_scaler, float motor2_speed_scaler, float motor3_speed_scaler, float motor4_speed_scaler) {
	this->motor1_speed_scaler = motor1_speed_scaler;
	this->motor2_speed_scaler = motor2_speed_scaler;
	this->motor3_speed_scaler = motor3_speed_scaler;
	this->motor4_speed_scaler = motor4_speed_scaler;
}

void QuadBaseKinematics::calculate_speeds(float current_angle, float velocity_x, float velocity_y, float omega) {
	if (current_angle == 0 && velocity_x == 0 && velocity_y == 0 && omega == 0){
		this->brake();
		return;
	}
	float s1 = (-sin(current_angle+alpha1)*cos(current_angle)*velocity_x) + (cos(current_angle+alpha1)*cos(current_angle)*velocity_y) + (distance1*omega);
	float s2 = (-sin(current_angle+alpha2)*cos(current_angle)*velocity_x) + (cos(current_angle+alpha2)*cos(current_angle)*velocity_y) + (distance2*omega);
	float s3 = (-sin(current_angle+alpha3)*cos(current_angle)*velocity_x) + (cos(current_angle+alpha3)*cos(current_angle)*velocity_y) + (distance3*omega);
	float s4 = (-sin(current_angle+alpha4)*cos(current_angle)*velocity_x) + (cos(current_angle+alpha4)*cos(current_angle)*velocity_y) + (distance4*omega);
	this->speed1 = -s1*motor1_speed_scaler;
	this->speed2 = -s2*motor2_speed_scaler;
	this->speed3 = -s3*motor3_speed_scaler;
	this->speed4 = -s4*motor4_speed_scaler;
}

void QuadBaseKinematics::brake() {
	this->speed1 = 0;
	this->speed2 = 0;
	this->speed3 = 0;
	this->speed4 = 0;
}
