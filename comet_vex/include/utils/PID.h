#pragma once

#include <chrono>

class PID
{
public:
    PID(double kP, double kI, double kD, double tolerance = 0.0)
        : kP(kP), kI(kI), kD(kD), tolerance(tolerance), lastError(0), integral(0)
    {
        lastTime = std::chrono::high_resolution_clock::now();
    }

    void setCoefficients(double kp, double ki, double kd)
    {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    void reset()
    {
        lastError = 0;
        integral = 0;
        lastTime = std::chrono::high_resolution_clock::now();
    }

    double calculate(double current, double target)
    {
        double error = target - current;
        return calculate(error);
    }

    double calculate(double error) {
        using namespace std::chrono;

        auto now = high_resolution_clock::now();
        double dt = duration<double>(now - lastTime).count();
        lastTime = now;

        if (dt <= 0.0)
        {
            dt = 1e-3;
        }

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;

        return output;
    }

    bool atTarget(double current, double target)
    {
        return std::abs(target - current) <= tolerance;
    }

private:
    double kP, kI, kD;
    double tolerance;
    double lastError;
    double integral;

    std::chrono::high_resolution_clock::time_point lastTime;
};