#ifndef INC_TRIBASE_H_
#define INC_TRIBASE_H_

#include "main.h"
#include "Motor.h"

class tribase_kinematics {
protected:
    MOTOR* motor_1;
    MOTOR* motor_2;
    MOTOR* motor_3;

public:
    // Constructor
    tribase_kinematics(MOTOR* motor_1, MOTOR* motor_2, MOTOR* motor_3);

    // Motor initialization
    void motor_init(TIM_HandleTypeDef* timer, uint32_t CHANNEL);

    // Movement functions
    void forward(uint8_t pwm_2, uint8_t pwm_3);
    void backward(uint8_t pwm_2, uint8_t pwm_3);
    void right(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void left(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void anti_clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);
    void clockwise(uint8_t pwm_1, uint8_t pwm_2, uint8_t pwm_3);

    // Conversion function
    float pwmToVelocity(int pwmValue);

    // Inverse kinematics function
    void IK(float omega_bz, float b_x, float b_y);

    // Brake function
    void brake();
};

#endif /* INC_TRIBASE_H_ */
