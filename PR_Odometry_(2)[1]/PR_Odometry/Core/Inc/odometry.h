/*
 * odometry.h
 *
 *  Created on: Dec 8, 2024
 *      Author: Hetvi
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "encoder.h"
#include "GY_25.h"

class Odometry {
private:
	 Encoder& x_encoder;
	 Encoder& y_encoder;

	 GY_25& imu;
	 float x;
	 float y;
     float theta;
	 float x_offset;
	 float y_offset;
     float prev_x_ticks;
     float prev_y_ticks;

public:
    Odometry(Encoder& x_encoder, Encoder& y_encoder, GY_25& imu, float x_offset , float y_offset );

    void init();
    void odometry();
    void update();
    float getYawFromIMU();
    void getValue(float& x_value, float& y_value, float& theta_value);
};



#endif /* INC_ODOMETRY_H_ */
