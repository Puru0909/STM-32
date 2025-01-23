#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "encoder.h"
#include "gy_25.h"
//#include "stm32f4xx_hal.h" // Include STM32 HAL library

class Odometry {
public:
    Odometry(TIM_HandleTypeDef* timer_handle_x, int pulses_per_rev_x, float diameter_x, uint8_t unit_x,
             TIM_HandleTypeDef* timer_handle_y, int pulses_per_rev_y, float diameter_y, uint8_t unit_y,
             GY_25& imu, float xOffset, float yOffset);

    void init();
    void update();

    float xPosition;
    float yPosition;
    float theta;

    float getXPosition();
    float getYPosition();
    float getTheta();

private:
    Encoder encoder_x;
    Encoder encoder_y;
    GY_25& imu;

    float xOffset;
    float yOffset;



    float rawTheta;
    float currentTheta;
    float lastTheta;
    float deltaTheta;

    float currentEncoderX;
    float currentEncoderY;
    float lastEncoderX;
    float lastEncoderY;
    float deltaX;
    float deltaY;

    float deltaPositionX;
    float deltaPositionY;
};

#endif // ODOMETRY_H
