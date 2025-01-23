#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "Encoder_v1.0.h"
#include "BNO085.h"
#include "stm32f4xx_hal.h"

class Odometry {
public:
    Odometry(Encoder& encoder_x, Encoder& encoder_y, Sensors::IMU& imu, float x_offset, float y_offset);

    void init();
    void update();

    Encoder& encoder_x;
    Encoder& encoder_y;
    Sensors::IMU& imu;

    float x_offset;
    float y_offset;

    float x_position;
    float y_position;
    float theta;

    float dt;
    float velocity_x;
	float velocity_y;
	float omega;

	float last_encoder_x;
    float last_encoder_y;
    float last_theta;
    uint8_t last_time;

    float current_encoder_x;
    float current_encoder_y;
    float current_theta;

    float delta_x;
    float delta_y;
    float delta_theta;

    float delta_position_x;
    float delta_position_y;

};

#endif // ODOMETRY_H
