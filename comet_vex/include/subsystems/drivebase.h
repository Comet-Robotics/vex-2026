#pragma once

#include "constants.h"
#include "utils/Twist2D.h"
#include "pros/motor_group.hpp"

using namespace constants::drivebase;
class Drivebase
{
    public:
        Drivebase() = default;
        
        void errorDrive(float drive, float turn) {
            drive /= 127.0;
            turn /= 127.0;

            turn /= ((drive < 0.5) ? 1.5 : 1.2);

            // int driveSign = ((drive >= 0)? 1 : -1);
            // int turnSign  = ((turn >= 0)?  1 : -1);

            // drive = driveSign * pow(drive, 2);
            // turn = turnSign * pow(turn, 2);


            // divide to normalize motor voltages? test both!

            LEFT_MOTORS.move_voltage((drive + turn) * 12000);
            RIGHT_MOTORS.move_voltage((drive - turn) * 12000);
        }

        void update() {
            calculateTwist();
        }

        Twist2D getTwist() {
            return twist;
        }

        void calibrateIMU() {
            IMU.reset(true);
            while (IMU.is_calibrating()) {
                pros::delay(10);
            }
        }

        // void goToPose(Pose pose)

    private:
        pros::MotorGroup LEFT_MOTORS{
            LEFT_PORTS,
            CHASSIS_INTERNAL_GEARSET
        };
        pros::MotorGroup RIGHT_MOTORS{
            RIGHT_PORTS,
            CHASSIS_INTERNAL_GEARSET
        };
        Twist2D twist;
        
        double previousHeading = IMU.get_heading();
        double previousTime = pros::millis();
        bool pastFirstLoop = false;
        void calculateTwist() {
            double rawLeftVel = LEFT_MOTORS.get_actual_velocity();
            double rawRightVel = RIGHT_MOTORS.get_actual_velocity();
            
            double dt = 50; // ms
            if (pastFirstLoop) {
                dt = pros::millis() - previousTime;
            } else {
                pastFirstLoop = true;
            }
            previousTime = pros::millis();

            double currentHeading = IMU.get_heading();
            double headingVel = (currentHeading - previousHeading) / (dt / 1000);

            double leftVel = rawLeftVel * 2 * 3.14159 * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);
            double rightVel = rawRightVel * 2 * 3.14159 * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);

            double vx = 0;
            double vy = leftVel + (rightVel - leftVel) / 2;
            double w = headingVel;

            twist = {vx, vy, w};
        }
};