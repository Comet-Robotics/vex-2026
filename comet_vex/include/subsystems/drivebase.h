#pragma once

#include "constants.h"
#include "utils/Twist2D.h"
#include "pros/motor_group.hpp"
#include <Pose2D.h>

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

        void setPose(Pose2D pose) {
            currentPose = pose;
        }

        void goToPose(Pose2D goal, uint32_t driveTimeout = 5000, uint32_t turnTimeout = 3000) {
            double kPLinear = 0.5;
            double kPAngular = 2.0;
            
            uint32_t startTime = pros::millis();

            // drive to position
            while (currentPose.distance(goal) > 1.0 && (pros::millis() - startTime) < driveTimeout) {
                Pose2D error = goal - currentPose;

                // project onto robot heading
                double driveError = error.x * cos(currentPose.theta) + error.y * sin(currentPose.theta);

                double angleError = atan2(error.y, error.x) - currentPose.theta;
                while(angleError > M_PI) angleError -= 2*M_PI;
                while(angleError < -M_PI) angleError += 2*M_PI;

                double drive = kPLinear * driveError;
                double turn  = kPAngular * angleError;

                errorDrive(drive, turn);

                pros::delay(10);
            }

            startTime = pros::millis();

            // turn to final angle
            while (fabs(currentPose.theta - goal.theta) > 0.1 && (pros::millis() - startTime) < turnTimeout) {
                double angleError = goal.theta - currentPose.theta;
                while(angleError > M_PI) angleError -= 2*M_PI;
                while(angleError < -M_PI) angleError += 2*M_PI;

                double turn  = kPAngular * angleError;

                errorDrive(0, turn);

                pros::delay(10);
            }

            // stop motors
            errorDrive(0, 0);
        }

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
        Pose2D currentPose;
        
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