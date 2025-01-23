#ifndef INC_QUADBASE_H_
#define INC_QUADBASE_H_

#include "math.h"

class QuadBaseKinematics {
public:
	QuadBaseKinematics(void);
	void set_speed_scalers(float motor1_speed_scaler, float motor2_speed_scaler, float motor3_speed_scaler, float motor4_speed_scaler);
    void calculate_speeds(float current_angle, float velocity_x, float velocity_y, float omega);
    void brake();

    float motor1_speed_scaler = 100;
    float motor2_speed_scaler = 100;
    float motor3_speed_scaler = 100;
    float motor4_speed_scaler = 100;

    float speed1;
    float speed2;
    float speed3;
    float speed4;

    float alpha1 = 3.5*2*3.14159265/4;
    float alpha2 = 2.5*2*3.14159265/4;
    float alpha3 = 0.5*2*3.14159265/4;
    float alpha4 = 1.5*2*3.14159265/4;

    float distance1 = 0.3125;
    float distance2 = 0.3125;
    float distance3 = 0.3125;
    float distance4 = 0.3125;
};

#endif  // INC_QUADBASE_H_
