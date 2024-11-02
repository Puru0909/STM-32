// PIDController.cpp
// PIDController.cpp
#include "PID.h"

// Constructor to initialize PID constants and state variables
PID::PID(float Kp, float Ki, float Kd, float dt)
    : Kp(Kp), Ki(Ki), Kd(Kd), dt(dt), prevError(0), integral(0) {}

// Compute the control signal based on the setpoint and measured value
float PID::find(float setpoint, float measuredValue) {
    float error = setpoint - measuredValue;
    integral += error * dt; // Accumulate the error
    float derivative = (error - prevError) / dt;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    prevError = error; // Update previous error for the next iteration
    return output;
}

// Setter methods to update PID constants
void PID::setKp(float Kp) {
	this->Kp = Kp;
}
void PID::setKi(float Ki) {
 this->Ki = Ki;
}
void PID::setKd(float Kd) {
	this->Kd = Kd;
}


// main.cpp
 // Include STM32 HAL libraries
#include "PID.h" // Include your custom PID library

int main(void) {
    // HAL initialization
    HAL_Init();
    SystemClock_Config(); // Assuming you have a function for clock config

    // Initialize the PID controller
    PIDController pid(1.0, 0.1, 0.05, 0.01); // Kp, Ki, Kd, and dt

    float setpoint = 100.0; // Desired value (e.g., target motor speed)
    float measuredValue = 0.0; // Actual value (e.g., measured motor speed)
    float controlSignal = 0.0;

    // Main loop
    while (1) {
        measuredValue = getMeasuredValue(); // Replace with actual sensor reading
        controlSignal = pid.compute(setpoint, measuredValue); // Calculate control signal
        applyControlSignal(controlSignal); // Replace with motor control function

        HAL_Delay(10); // 10ms delay (matching dt)
    }
}

