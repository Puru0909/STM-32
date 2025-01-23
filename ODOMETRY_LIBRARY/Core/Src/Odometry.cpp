#include "Odometry.h"
#include "Encoder.h"
#include "math.h"
#include <cmath>

Odometry::Odometry(encoder* leftEncoder, encoder* rightEncoder, float wheelRadius, int cpr, double Xoffset, double Yoffset) {
    this->leftEncoder = leftEncoder;
    this->rightEncoder = rightEncoder;
    this->wheelRadius = wheelRadius;
    this->Xoffset = Xoffset;
    this->Yoffset = Yoffset;
    this->cpr = cpr;
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->prevLeftRevolutions = 0;
    this->prevRightRevolutions = 0;
    this->radian_per_tick = (2 * M_PI * wheelRadius) / cpr;
}

void Odometry::update_position() {


    // Get current revolutions from encoders
    int16_t leftRevolutions = leftEncoder->read_revolutions();
    int16_t rightRevolutions = rightEncoder->read_revolutions();
// Calculate change in revolutions
    int16_t deltaLeftRevolutions = leftRevolutions - prevLeftRevolutions;
    int16_t deltaRightRevolutions = rightRevolutions - prevRightRevolutions;

   // Update previous revolutions
    prevLeftRevolutions =  leftRevolutions;
    prevRightRevolutions = rightRevolutions;


   // float radian_per_tick = (2 * M_PI * wheelRadius) / cpr;

    // Convert revolutions to distance (arc_length)

    float deltaLeftDistance = deltaLeftRevolutions * (radian_per_tick + Xoffset * theta);
    float deltaRightDistance = deltaRightRevolutions *(radian_per_tick - Yoffset * theta);
//  float deltaLeftDistance = deltaLeftRevolutions * radian_per_tick;
//  float deltaRightDistance = deltaRightRevolutions * radian_per_tick;

//  Calculate average distance traveled
    float deltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;

    // Calculate change in orientation (theta)
    float deltaTheta = (deltaRightDistance - deltaLeftDistance) / 0.125; // Assuming wheel distance between the two encoders is 0.125 meters

    // Update position and orientation

    x += deltaLeftDistance * cos(theta) - deltaRightDistance * sin(theta);
    y += deltaLeftDistance * sin(theta) + deltaRightDistance * cos(theta);
    theta += deltaTheta;
//     x += deltaDistance * cos(theta);
//     y += deltaDistance * sin(theta);
}


float Odometry::get_x() {
    return x;
}

float Odometry::get_y() {
    return y;
}

float Odometry::get_theta() {
    return theta;
}
