#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "Encoder.h"
#include "main.h"
#include <cmath>

class Odometry {
public:
    Odometry(encoder* leftEncoder, encoder* rightEncoder, float wheelRadius, int cpr, double Xoffset, double Yoffset);
    void  update_position();
    float get_x();
    float get_y();
    float get_theta();

private:
    encoder* leftEncoder;
    encoder* rightEncoder;
    float wheelRadius;  // Radius of the wheels (in m)
    int cpr;            // countes per revolution
    float Xoffset;
    float Yoffset;


    float x;            // Current x position (in meters)
    float y;            // Current y position (in meters)
    float theta;        // Orientation in radians
    float radian_per_tick;
    int16_t prevLeftRevolutions;
    int16_t prevRightRevolutions;
};

#endif /* INC_ODOMETRY_H_ */
