#include "PID.h"

PID::PID(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    integral = 0.0;
    prevError = 0.0;
    prevTime = HAL_GetTick(); // Initialize the previous time using HAL_GetTick
}

double PID::compute(double error) {
    uint32_t currentTime = HAL_GetTick(); // Get the current time in milliseconds
    double deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds

    // Proportional term
    double proportional = kp * error;

    // Integral term
    integral += error * deltaTime;
    double integralTerm = ki * integral;

    // Derivative term
    double derivative = (error - prevError) / deltaTime;
    double derivativeTerm = kd * derivative;

    // PID output
    double output = proportional + integralTerm + derivativeTerm;

    // Update for next iteration
    prevError = error;
    prevTime = currentTime;

    return output;
}

void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
    prevTime = HAL_GetTick(); // Reset the time
}
