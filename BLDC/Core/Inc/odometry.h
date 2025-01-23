/*
 * odometry.h
 *
 *  Created on: Dec 21, 2024
 *      Author: Admin
 */

#ifndef ODOMETERY_H
#define ODOMETERY_H

#include <math.h>
#include "encoder.h"

enum ODOM_Status
{
    ODOM_PASS,
    ODOM_FAIL
};

typedef struct __Pose2D
{
    float x, y, z;
} Pose2D;

typedef struct __Velocity2D
{
    float Vx, Vy, Vz;
} Velocity2D;

namespace Localization
{
    class Odometry
    {
    protected:
        Pose2D pose;
        Velocity2D speed;
        bool run = true;
        Encoder *encoderX, *encoderY;
        float radius = 0.029, factorX = 0, factorY = 0; // radius is always in meters
        float currX = 0, currY = 0, prevX = 0, prevY = 0, prevZ = 0;
        float diffX = 0, diffY = 0, diffZ = 0, currZ = 0;
        float x_offset = 0.26, y_offset = -0.26;
        float xLocal, yLocal;
    public:
        Odometry(Encoder *encoderX, Encoder *encoderY, float radius);
        void set_offsets(float x_offset, float y_offset);
        float get_diffZ() { return diffZ; };
        Pose2D get_position(float pose_z);
        Velocity2D get_speed(float dT);
        void set_position(Pose2D pose);

    };
};

#endif
