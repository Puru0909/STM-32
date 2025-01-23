/*
 * odometry.cpp
 *
 *  Created on: Dec 8, 2024
 *      Author: Hetvi
 */

#include "odometry.h"
#include <cmath>
#define PI 3.14159265358979323846f

Odometry::Odometry(Encoder& x_encoder, Encoder& y_encoder, GY_25& imu, float x_offset, float y_offset)
    : x_encoder(x_encoder), y_encoder(y_encoder),imu(imu), x_offset(x_offset), y_offset(y_offset){

}

void Odometry::init(){
	x_encoder.reset();
	y_encoder.reset();
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    vx = 0.0f;
    vy = 0.0f;
    vz = 0.0f;
}

float Odometry::getYawFromIMU(){
	return imu.x_angle;
}

void Odometry::update(){

	prev_x_ticks = current_x_ticks;
	prev_y_ticks = current_y_ticks;
	prev_theta = current_theta;
	prev_time = HAL_GetTick();

    current_x_ticks = x_encoder.getDistance(ENCODER_UNITS.METER);
    current_y_ticks = y_encoder.getDistance(ENCODER_UNITS.METER);
    current_theta = (imu.x_angle)*2*M_PI/360;

    float delta_x_ticks = current_x_ticks - prev_x_ticks;
    float delta_y_ticks = current_y_ticks - prev_y_ticks;
    float delta_theta = current_theta - prev_theta;

    uint32_t current_time = HAL_GetTick();
    float delta_t = (current_time - prev_time)/1000;
    prev_time = current_time;

    float delta_x = delta_x_ticks + ( x_offset * delta_theta);
    float delta_y = delta_y_ticks + ( y_offset * delta_theta);

    z += delta_theta;
    x += delta_x * cos(z) - delta_y * sin(z);
    y += delta_x * sin(z) + delta_y * cos(z);

    vx = delta_x / delta_t;
    vy = delta_y / delta_t;
    vz = delta_theta / delta_t;
}

float Odometry::getX(){
    return x;
}
float Odometry::getY(){
    return y;
}
float Odometry::getZ(){
    return z;
}
float Odometry::get_vx(){
	return vx;
}
float Odometry::get_vy(){
	return vy;
}
float Odometry::get_vz(){
	return vz;
}



