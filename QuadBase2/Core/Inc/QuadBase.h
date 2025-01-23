#ifndef INC_QUADBASE_H_
#define INC_QUADBASE_H_

#include "math.h"

class QuadBaseKinematics {
public:
	QuadBaseKinematics(void);
	void set_speed_scalers(float* motor_speed_scalers);
    void calculate_speeds(float current_angle, float change_in_x, float change_in_y, float change_in_angle);
    void brake();

    float motor1_base_speed;
    float motor2_base_speed;
    float motor3_base_speed;
    float motor4_base_speed;

    float* speeds;

    float alpha1 = 0*2*3.14159265/4;
    float alpha2 = 1*2*3.14159265/4;
    float alpha3 = 2*2*3.14159265/4;
    float alpha4 = 3*2*3.14159265/4;
};

#endif  // INC_QUADBASE_H_
