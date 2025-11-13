#pragma once

// PID Controller Class: Provides proportional-integral-derivative control logic
class PID
{
private:
    // PID gain constants: tune these for your system
    double kP, kI, kD;

    // State variables for integral and derivative calculations
    double previousError = 0.0;
    double integral = 0.0;

    // Output clamping to prevent actuator saturation or windup
    double outputMin = -1e6, outputMax = 1e6;

public:
    // Initialize PID controller with gain constants
    PID(double kP, double kI, double kD)
        : kP(kP), kI(kI), kD(kD) {}

    // Update PID gain constants at runtime (for adaptive tuning)
    void setConstants(double kP, double kI, double kD)
    {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    // Set minimum and maximum output values (prevents excessive control effort)
    void setOutputLimits(double min, double max)
    {
        outputMin = min;
        outputMax = max;
    }

    // Reset integral and previous error (useful when starting or re-initializing control)
    void reset()
    {
        integral = 0.0;
        previousError = 0.0;
    }

    // Compute PID output based on setpoint and measured value
    //   setpoint:      Desired target value
    //   measuredValue: Current process variable
    //   dt:            Time step (seconds); affects integral/derivative scaling
    // Returns:         Control output (e.g., motor command)
    double calculate(double setpoint, double measuredValue, double dt = 1.0)
    {
        // Calculate error between desired and actual value
        double error = setpoint - measuredValue;

        // Accumulate integral (sum of errors over time)
        integral += error * dt;

        // Calculate rate of error change (derivative)
        double derivative = (error - previousError) / dt;

        // Weighted sum of P, I, D terms
        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp output to specified limits to avoid actuator saturation
        if (output > outputMax)
            output = outputMax;
        if (output < outputMin)
            output = outputMin;

        // Store current error for next derivative calculation
        previousError = error;
        return output;
    }
};
