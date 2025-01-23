#include "PID.h"

PID::PID(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    integral = 0;
    prevError = 0;
    prevTime = HAL_GetTick();
}

double PID::compute(double error) {
    unsigned long currentTime = HAL_GetTick();
    double deltaTime = (currentTime - prevTime) / 1000.0;

    double proportional = kp * error;

    integral += error * deltaTime;
    double integralTerm = ki * integral;

    double derivative = (error - prevError) / deltaTime;
    double derivativeTerm = kd * derivative;

    double output = proportional + integralTerm + derivativeTerm;

    prevError = error;
    prevTime = currentTime;

    return output;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    prevTime = HAL_GetTick();
}
