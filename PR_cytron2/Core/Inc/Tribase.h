#ifndef TRIBASEKINEMATICS_H
#define TRIBASEKINEMATICS_H

#include "math.h"
#include "CYTRON.h"

class TriBase {
public:
    // Constructor and Destructor
    TriBase(Cytron& motor1, Cytron& motor2, Cytron& motor3);
    void set_base_speeds(float motor1_base_speed, float motor2_base_speed, float motor3_base_speed);

    void update_motor_speeds();
    void go_at_angle(float current_angle, float change_in_x, float change_in_y, float change_in_angle);
    void brake();

    float motor1_base_speed;
    float motor2_base_speed;
    float motor3_base_speed;

    float speed1;
    float speed2;
    float speed3;

    float alpha1 = 0;
    float alpha2 = 2*3.14159265/3;
    float alpha3 = 4*3.14159265/3;

    Cytron motor1;
    Cytron motor2;
    Cytron motor3;
};

#endif  // TRIBASEKINEMATICS_H
