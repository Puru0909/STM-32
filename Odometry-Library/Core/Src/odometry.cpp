#include "odometry.h"

Odometry::Odometry(TIM_HandleTypeDef* timer_handle_x, int pulses_per_rev_x, float diameter_x, uint8_t unit_x,
                   TIM_HandleTypeDef* timer_handle_y, int pulses_per_rev_y, float diameter_y, uint8_t unit_y,
                   GY_25& imu, float xOffset, float yOffset)
    : encoder_x(timer_handle_x, pulses_per_rev_x, diameter_x, unit_x),
      encoder_y(timer_handle_y, pulses_per_rev_y, diameter_y, unit_y),
      imu(imu),
      xOffset(xOffset), yOffset(yOffset) {}

void Odometry::init() {
    encoder_x.init();
    encoder_y.init();

    xPosition = 0.0f;
    yPosition = 0.0f;
    theta = 0.0f;
}

void Odometry::update() {

    imu.update_counts();
    imu.update_angles();


    encoder_x.update();
    encoder_y.update();

    lastEncoderX = currentEncoderX;
    lastEncoderY = currentEncoderY;
    lastTheta = currentTheta;

    currentEncoderX = encoder_x.getDistance(ENCODER_UNITS.METER);
    currentEncoderY = encoder_y.getDistance(ENCODER_UNITS.METER);
    currentTheta = imu.x_angle * 2 * M_PI / 360.0f;


    deltaTheta = currentTheta - lastTheta;
    deltaX = currentEncoderX - lastEncoderX;
    deltaY = currentEncoderY - lastEncoderY;

    deltaPositionX = deltaX + (xOffset * deltaTheta);
    deltaPositionY = deltaY + (yOffset * deltaTheta);

    xPosition += cos(theta) * deltaPositionX - sin(theta) * deltaPositionY;
    yPosition += sin(theta) * deltaPositionX + cos(theta) * deltaPositionY;
    theta = currentTheta;
}

float Odometry::getXPosition() {
    return xPosition;
}

float Odometry::getYPosition() {
    return yPosition;
}

float Odometry::getTheta() {
    return theta;
}
