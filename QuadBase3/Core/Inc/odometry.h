/*
 * odometry.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Hetvi
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "Encoder_v1.0.h"
#include "GY_25.h"

class Odometry {
public:
	 Encoder& x_encoder;
	 Encoder& y_encoder;

	 GY_25& imu;
	 float x;
	 float y;
	 float z;
	 float vx;
	 float vy;
	 float vz;
     float prev_theta;
	 float x_offset;
	 float y_offset;
	 float current_x_ticks;
	 float current_y_ticks;
	 float current_theta;
     float prev_x_ticks;
     float prev_y_ticks;
     uint32_t prev_time;

    Odometry(Encoder& x_encoder, Encoder& y_encoder, GY_25& imu, float x_offset , float y_offset );

    void init();
    void odometry();
    void update();
    float getYawFromIMU();
    float getX();
    float getY();
    float getZ();
    float get_vx();
    float get_vy();
    float get_vz();
};

#endif /* INC_ODOMETRY_H_ */
