// PIDController.h
#ifndef PID_H
#define PID_H

class PID {
private:
    // PID coefficients
    float Kp, Ki, Kd;

    // PID state variables
    float prevError;
    float integral;
    float dt; // Time step

public:
    // Constructor
    PID(float Kp, float Ki, float Kd, float dt);

    // Method to compute the control signal
    float find(float setpoint, float measuredValue);

    // Optional: Setter methods to update PID constants
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
};

#endif // PIDCONTROLLER_H
