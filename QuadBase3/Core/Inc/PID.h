#ifndef PID_H
#define PID_H

#include "main.h"
#include "float.h"
#include "stdlib.h"

class PID {
private:
    double kp;
    double ki;
    double kd;

    double integral;

    double prevError;
    unsigned long prevTime;

    double max_output = 1000000000;
    double min_output = -1000000000;
    double minimum_error = 0;

public:
    PID(double Kp, double Ki, double Kd);

    void set_minimum_error(double minimum_error);

    void set_output_constrains(double max);

    void set_output_constrains(double max, double min);

    double compute(double error);

    void reset();
};

#endif
