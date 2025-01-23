/*
 * odometry.cpp
 *
 *  Created on: Dec 21, 2024
 *      Author: Admin
 */


#include "odometry.h"

Localization::Odometry::Odometry(Encoder *encoderX, Encoder *encoderY, float radius)
{
    this->encoderX = encoderX;
    this->encoderY = encoderY;
    speed = {0, 0, 0};
    this->radius = radius;
    pose = {0, 0, 0};
    factorX = (2 * M_PI * this->radius) / 1600;
    factorY = (2 * M_PI * this->radius) / 400;
}

void Localization::Odometry::set_position(Pose2D pose_) {


	this->pose = pose_;
	// xLocal = pose.x * cos(pose.z) + pose.y * sin(pose.z);
	// yLocal = pose.x * -sin(pose.z) + pose.y * cos(pose.z);
	// this->encoderX->write((long)(xLocal * 400));
	// this->encoderY->write((long)(yLocal * 400));
}


void Localization::Odometry::set_offsets(float x_offset, float y_offset)
{
    this->x_offset = x_offset;
    this->y_offset = y_offset;
}

Pose2D Localization::Odometry::get_position(float pose_z)
{
    prevX = currX;
    prevY = currY;
    prevZ = pose.z;
    currZ = pose_z * M_PI / 180.0;
    currX = encoderX->encoder_read();
    currY = encoderY->encoder_read();
    if(abs(pose.z - prevZ) < 2){
      diffZ = (currZ - prevZ);
      pose.z += diffZ;
    }
    diffX = (currX - prevX) * factorX - x_offset * diffZ;
    diffY = (currY - prevY) * factorY - y_offset * diffZ;
    pose.x += diffX * cos(pose.z) - diffY * sin(pose.z);
    pose.y += diffX * sin(pose.z) + diffY * cos(pose.z);
    return pose;
}

Velocity2D Localization::Odometry::get_speed(float dT)
{
    speed.Vx = diffX / dT;
    speed.Vy = diffY / dT;
    speed.Vz = diffZ / dT;
    return speed;
}
