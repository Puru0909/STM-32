#include "PID.h"

PID::PID(double Kp, double Ki, double Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    integral = 0;
    prevError = 0;
    prevTime = HAL_GetTick();
}

void PID::set_minimum_error(double minimum_error) {
	this->minimum_error = minimum_error;
}

void PID::set_output_constrains(double max) {
	this->max_output = max;
	this->min_output = -max;
}

void PID::set_output_constrains(double max, double min) {
	this->max_output = max;
	this->min_output = min;
}

double PID::compute(double error) {
    unsigned long currentTime = HAL_GetTick();
	if (abs(error) <= minimum_error) {
	    prevError = 0;
	    prevTime = currentTime;
	    return 0;
	}
    double deltaTime = (currentTime - prevTime) / 1000.0;

    double proportional = kp * error;

    integral += error * deltaTime;
    double integralTerm = ki * integral;

    double derivative = (error - prevError) / deltaTime;
    double derivativeTerm = kd * derivative;

    double output = proportional + integralTerm + derivativeTerm;

    prevError = error;
    prevTime = currentTime;

    if (output < min_output) {
    	output = min_output;
    }
    else if (output > max_output) {
    	output = max_output;
    }

    return output;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
    prevTime = HAL_GetTick();
}
