#ifndef PID_H
#define PID_H

#include "main.h"

class PID {
private:
    double kp;
    double ki;
    double kd;
    double integral;
    double prevError;
    unsigned long prevTime;
public:
    PID(double Kp, double Ki, double Kd);

    double compute(double error);

    void reset();
};

#endif
