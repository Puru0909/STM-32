/*
 * PID.h
 *
 *  Created on: Nov 30, 2024
 *      Author: Dell
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h" // Include HAL and necessary system headers

class PID {
private:
    double kp;        // Proportional gain
    double ki;        // Integral gain
    double kd;        // Derivative gain
    double integral;  // Accumulated integral term
    double prevError; // Previous error
    uint32_t prevTime; // Previous timestamp in milliseconds

public:
    // Constructor to initialize gains
    PID(double Kp, double Ki, double Kd);

    // Compute the PID output based on the error
    double compute(double error);

    // Reset the PID terms
    void reset();
};

#endif

 /* INC_PID_H_ */
