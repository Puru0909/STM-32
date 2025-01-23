/*
 * odometry.cpp
 *
 *  Created on: Dec 8, 2024
 *      Author: Hetvi
 */

#include "main.h"
#include "odometry.h"
#include "encoder.h"
#include "GY_25.h"
#include "math.h"
#define PI 3.14159265358979323846f

Odometry::Odometry(Encoder& x_encoder, Encoder& y_encoder, GY_25& imu, float x_offset, float y_offset)
    : x_encoder(x_encoder), y_encoder(y_encoder),imu(imu), x_offset(x_offset), y_offset(y_offset) {

}

void Odometry::init() {
	x_encoder.reset();
	y_encoder.reset();
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
    prev_x_ticks = 0;
    prev_y_ticks = 0;
}

float Odometry::getYawFromIMU(){
	return imu.x_angle;
}


void Odometry::update() {

    float current_x_ticks = x_encoder.getTotalCount();
    float current_y_ticks = y_encoder.getTotalCount();

    float delta_x_ticks = current_x_ticks - prev_x_ticks;
    float delta_y_ticks = current_y_ticks - prev_y_ticks;

    prev_x_ticks = current_x_ticks;
    prev_y_ticks = current_y_ticks;

    float new_theta = getYawFromIMU();
    float delta_theta = new_theta - theta;

    theta = new_theta;

    float rad_per_tick = (2 * PI * x_encoder.diameter_in_meter / 2) / 2400;

    float delta_x = delta_x_ticks * (rad_per_tick - x_offset * delta_theta);
    float delta_y = delta_y_ticks * (rad_per_tick + y_offset * delta_theta);

    x += (delta_x * cos(theta)) - (delta_y * sin(theta));
    y += (delta_x * sin(theta)) + (delta_y * cos(theta));
}

void Odometry::getValue(float& x_value, float& y_value, float& theta_value) {
    x_value = x;
    y_value = y;
    theta_value = theta;
}

