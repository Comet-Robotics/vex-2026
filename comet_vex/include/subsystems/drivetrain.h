#pragma once

// -------------------- Includes --------------------
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp" // for pros::delay
#include <cmath>

// -------------------- Drivetrain Class --------------------
class Drivetrain
{
private:
    pros::MotorGroup leftMotors;
    pros::MotorGroup rightMotors;
    pros::IMU imu;

    // drivetrain geometry constants
    const double wheelDiameter = 4.125; // inches
    const double wheelCircumference = wheelDiameter * M_PI;
    const double gearRatio = 1.0; // adjust if using gearing

public:
    // -------------------- Constructor --------------------
    Drivetrain()
        : leftMotors({}), rightMotors({}), imu(0) // Fill motor ports later
    {
        // calibrate IMU
        imu.reset();

        // wait for calibration to finish
        while (imu.is_calibrating())
        {
            pros::delay(50);
        }
    }

    // -------------------- Encoder Reset --------------------
    void resetEncoders()
    {
        leftMotors.tare_position();
        rightMotors.tare_position();
    }

    // -------------------- Move a Distance --------------------
    // Moves forward a given number of inches (open-loop, no PID yet)
    void moveInches(double inches, int voltage = 8000)
    {
        resetEncoders();
        double targetDegrees = (inches / wheelCircumference) * 360.0 * gearRatio;

        while (true)
        {
            double leftPos = std::abs(leftMotors.get_position());
            double rightPos = std::abs(rightMotors.get_position());

            if (leftPos >= targetDegrees || rightPos >= targetDegrees)
                break;

            moveVoltage(voltage, voltage);
            pros::delay(10);
        }
        moveVoltage(0, 0);
    }

    // -------------------- Get IMU Angle --------------------
    inline double getAngle()
    {
        return imu.get_heading(); // returns 0–360 degrees
    }

    // -------------------- Turn by Angle --------------------
    // Turns robot by a certain number of degrees (open-loop)
    void turnAngle(double angle, int voltage = 8000)
    {
        double startAngle = getAngle();
        double targetAngle = startAngle + angle;

        // Normalize to 0–360 range
        if (targetAngle >= 360)
            targetAngle -= 360;
        if (targetAngle < 0)
            targetAngle += 360;

        // Turn until near target
        while (std::fabs(getAngle() - targetAngle) > 2.0)
        {
            if (angle > 0)
            { // turn right
                moveVoltage(voltage, -voltage);
            }
            else
            { // turn left
                moveVoltage(-voltage, voltage);
            }
            pros::delay(10);
        }
        moveVoltage(0, 0);
    }

    // -------------------- Drive Straight --------------------
    // Basic forward drive (scaled by joystick input)
    // Accepts value between -1.0 to 1.0
    inline void drive(float forward)
    {
        moveVoltage(forward * 12000, forward * 12000);
    }

    // -------------------- Voltage Control --------------------
    void moveVoltage(int leftVoltage, int rightVoltage)
    {
        leftMotors.move_voltage(leftVoltage);
        rightMotors.move_voltage(rightVoltage);
    }

    // -------------------- Encoder Feedback --------------------
    inline double getLeftPosition() { return leftMotors.get_position(); }
    inline double getRightPosition() { return rightMotors.get_position(); }
};
