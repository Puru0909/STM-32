#include "Odometry.h"

Odometry::Odometry(Encoder& encoder_x, Encoder& encoder_y, Sensors::IMU& imu, float x_offset, float y_offset)
    : encoder_x(encoder_x), encoder_y(encoder_y), imu(imu), x_offset(x_offset), y_offset(y_offset){}

void Odometry::init() {
    encoder_x.init();
    encoder_y.init();

    x_position = 0.0f;
    y_position= 0.0f;
    theta = 0.0f;
}

void Odometry::update() {

    imu.update();

    encoder_x.update();
    encoder_y.update();

    uint32_t current_time = HAL_GetTick();

    float dt = (current_time - last_time) / 1000.0f;

    last_encoder_x = current_encoder_x;
	last_encoder_y = current_encoder_y;
	last_theta = current_theta;

	current_encoder_x = encoder_x.getDistance(ENCODER_UNITS.METER);
	current_encoder_y = encoder_y.getDistance(ENCODER_UNITS.METER);
	current_theta = -(imu.yaw)*2*M_PI/360;

	delta_theta = current_theta - last_theta;
	delta_x = current_encoder_x - last_encoder_x;
	delta_y = current_encoder_y - last_encoder_y;

    delta_position_x = delta_x - (delta_theta * x_offset);
	delta_position_y = delta_y - (delta_theta * y_offset);
	delta_position_x *= -1;
	dt = (current_time - last_time) / 1000.0f;

	x_position += cos(current_theta)*delta_position_x - sin(current_theta)*delta_position_y;
	y_position += sin(current_theta)*delta_position_x + cos(current_theta)*delta_position_y;
	theta = current_theta;

	velocity_x = delta_position_x / dt;  // m/s
	velocity_y = delta_position_y / dt;  // m/s
	omega = delta_theta / dt;  // rad/s

	last_time = current_time;
}
