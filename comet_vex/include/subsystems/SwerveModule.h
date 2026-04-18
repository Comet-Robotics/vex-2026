#pragma once

#include "constants.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "utils/MathUtils.h"
#include "utils/PID.h"
#include "utils/AngleUtils.h"
#include <cmath>
#include <array>
// #include "pros/llemu.hpp"

using namespace constants::drivetrain;

class SwerveModule
{
public:
    pros::Motor topMotor, bottomMotor;

    SwerveModule(int8_t topMotorPort, int8_t bottomMotorPort, int8_t rotationSensorPort) : rotationPID(0, 0, 0, 0),
                                                                                           topMotor(topMotorPort, CHASSIS_INTERNAL_GEARSET),
                                                                                           bottomMotor(bottomMotorPort, CHASSIS_INTERNAL_GEARSET),
                                                                                           rotationSensor(rotationSensorPort)
    {
        topMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        bottomMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        topMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        bottomMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    }

    double getModuleRotation() const
    {
        if (rotationSensor.get_port() == 0 || !constants::drivetrain::USE_ROTATION_SENSORS)
        {
            // calculate from motor encoders if no rotation sensor is set
            return ROTATION_FACTOR * (topMotor.get_position() - bottomMotor.get_position());
        }
        return rotationSensor.get_angle() / 100.0; // rotation sensor returns angle in hundredths of a degree
    }

    double getLinearSpeed() const
    {
        return currentLinearSpeed;
    }

    void calculateLinearSpeed()
    {
        double currentDistance = getDistanceInches();
        uint32_t currentTime = pros::millis();
        double deltaDistance = currentDistance - lastDistance;
        double deltaTime = (currentTime - lastTime) / 1000.0; // convert ms to seconds

        if (deltaTime <= 0.0)
        {
            deltaTime = 0.02; // default to 20 ms if time is invalid
        }

        currentLinearSpeed = deltaDistance / deltaTime; // inches per second

        lastDistance = currentDistance;
        lastTime = currentTime;
    }

    double getDistanceInches() const
    {
        double motorPos = (topMotor.get_position() + bottomMotor.get_position()) / 2.0;
        return LINEAR_FACTOR * GEAR_RATIO * (motorPos / 360.0) * (M_PI * WHEEL_DIAMETER); // inches
    }

    void setSpeedAndAngle(double targetSpeed, double targetAngle)
    {
        double currentAngle = getModuleRotation();
        AngleUtils::optimizeAngleAndSpeed(currentAngle, targetAngle, targetSpeed);

        // scale speed by cosine of angle delta to prevent strafing at high speeds when wheels are not aligned with movement direction
        if (constants::drivetrain::COSINE_SCALING)
        {
            double deltaRad = AngleUtils::toRadians(AngleUtils::shortestAngleDelta(currentAngle, targetAngle));
            targetSpeed *= cos(deltaRad);
        }

        speed = targetSpeed / MAX_LINEAR_SPEED; // scale to -1 to 1
        angle = targetAngle;
    }

    void setPID(const std::array<double, 3> &coefficients)
    {
        rotationPID.setCoefficients(coefficients[0], coefficients[1], coefficients[2]);
    }

    void setSpeeds(double topSpeed, double bottomSpeed)
    {
        topMotor.move_voltage(topSpeed * 12000);
        bottomMotor.move_voltage(bottomSpeed * 12000);
    }

    void update()
    {
        double currentAngle = AngleUtils::wrap360(getModuleRotation());
        double rotationPower = rotationPID.calculate(AngleUtils::shortestAngleDelta(currentAngle, angle));

        double topPower = speed + rotationPower;
        double bottomPower = speed - rotationPower;

        MathUtils::scaleToUnitRange(topPower, bottomPower);

        topMotor.move_voltage(topPower * 12000);
        bottomMotor.move_voltage(bottomPower * 12000);

        calculateLinearSpeed();
    }

    std::array<double, 2> getState() const
    {
        return {getLinearSpeed(), AngleUtils::wrap360(getModuleRotation())};
    }

    bool isOverTemperature() const
    {
        return topMotor.get_temperature() >= constants::MOTOR_TEMPERATURE_THRESHOLD ||
               bottomMotor.get_temperature() >= constants::MOTOR_TEMPERATURE_THRESHOLD;
    }

    void overCurrentDetector() const
    {
        if (topMotor.is_over_current())
        {
            printf("Motor %d is over current\n", topMotor.get_port());
        }
        if (bottomMotor.is_over_current())
        {
            printf("Motor %d is over current\n", bottomMotor.get_port());
        }
    }

private:
    pros::Rotation rotationSensor;
    bool fieldCentric;
    PID rotationPID;

    double speed = 0;
    double angle = 0;

    double currentLinearSpeed = 0.0;
    double lastDistance = 0.0;
    uint32_t lastTime = pros::millis();
};