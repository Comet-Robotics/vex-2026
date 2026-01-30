#pragma once

#include "constants.h"
#include "utils/Twist2D.h"
#include "pros/motor_group.hpp"
#include "utils/Pose2D.h"
#include "pros/llemu.hpp"
#include "utils/Math.h"
#include "utils/PID.h"

using namespace constants::drivebase;
class Drivebase
{
    public:
        Drivebase() {
            setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        }

        void setBrakeMode(pros::motor_brake_mode_e_t mode) {
            LEFT_MOTORS.set_brake_mode(mode);
            RIGHT_MOTORS.set_brake_mode(mode);
        }
        
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

        void normalDrive(float drive, float turn) {
            float max = fabs(drive) + fabs(turn);
            if (max > 1.0) {
                drive /= max;
                turn  /= max;
            }

            LEFT_MOTORS.move_voltage((drive + turn) * 12000);
            RIGHT_MOTORS.move_voltage((drive - turn) * 12000);
        }

        void update() {
            calculateTwist();
            updateLocalization();
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
            PID linear(0.02, 0.0001, 0.0005);
            PID angular(0.014, 0.001, 0.0003);
            
            uint32_t startTime = pros::millis();

            int count = 0;

            // turn to face goal
            while (fabs(atan2(goal.x - currentPose.x, goal.y - currentPose.y) * 180.0 / M_PI - currentPose.theta) > 0.3 && (pros::millis() - startTime) < turnTimeout) {
                double angleError = atan2(goal.x - currentPose.x, goal.y - currentPose.y) * 180.0 / M_PI - currentPose.theta;
                normalizeAngleDeg(angleError);

                double turn = angular.update(0, -angleError);
                normalDrive(0, turn);

                updateLocalization();

                pros::delay(50);
            }

            normalDrive(0, 0);

            angular.reset();

            // drive to position
            while (currentPose.distance(goal) > 0.5 && (pros::millis() - startTime) < driveTimeout) {
                count++;
                Pose2D error = goal - currentPose;
                
                pros::lcd::print(1, "error x: %f", error.x); // 0.5
                pros::lcd::print(2, "error y: %f", error.y); // 24
                pros::lcd::print(3, "error theta: %f", error.theta); // 0.2
                pros::lcd::print(4, "current theta: %f", currentPose.theta); // 0.2
                pros::lcd::print(5, "count: %d", count); // 0.2

                // project onto robot heading
                double driveError = error.x * sin(degToRad(currentPose.theta)) + error.y * cos(degToRad(currentPose.theta));

                double angleError = radToDeg(atan2(error.x, error.y)) - currentPose.theta;
                normalizeAngleDeg(angleError);

                double drive = linear.update(0, -driveError);
                double turn  = angular.update(0, -angleError);

                normalDrive(drive, turn);

                updateLocalization();

                pros::delay(50);
            }

            normalDrive(0, 0);

            pros::delay(500);

            startTime = pros::millis();

            angular.reset();

            // turn to final angle
            while (fabs(currentPose.theta - goal.theta) > 0.3 && (pros::millis() - startTime) < turnTimeout) {
                double angleError = goal.theta - currentPose.theta;
                normalizeAngleDeg(angleError);

                double turn = angular.update(0, -angleError);

                normalDrive(0, turn);

                updateLocalization();

                pros::delay(50);
            }

            // stop motors
            normalDrive(0, 0);
        }

        void updateLocalization() {
            // 1. Get current sensor values
            double leftPos = LEFT_MOTORS.get_position();   // degrees
            double rightPos = RIGHT_MOTORS.get_position(); // degrees
            
            // Use get_rotation() instead of get_heading() to get continuous values 
            // (e.g., 365 degrees instead of 5 degrees). This prevents math errors 
            // when crossing 0/360.
            double currentRotation = IMU.get_rotation();

            // 2. Handle Initialization
            if (!locInitialized) {
                prevLeftPos = leftPos;
                prevRightPos = rightPos;
                prevRotation = currentRotation; // You need to add this variable to your global/class state
                locInitialized = true;
                return;
            }

            // 3. Calculate Changes (Deltas)
            double dLeft = ticksToDistance(leftPos - prevLeftPos);
            double dRight = ticksToDistance(rightPos - prevRightPos);
            
            // Calculate raw linear distance
            double ds = (dLeft + dRight) / 2.0;

            // 4. Update Previous Values immediately
            prevLeftPos = leftPos;
            prevRightPos = rightPos;

            // 5. Calculate Heading for the Arc
            // We need the average heading during this specific movement step, 
            // not the heading at the end of the step.
            double prevThetaRad = prevRotation * (M_PI / 180.0);
            double currThetaRad = currentRotation * (M_PI / 180.0);
            double avgThetaRad = (prevThetaRad + currThetaRad) / 2.0;
            
            prevRotation = currentRotation;
            
            currentPose.x += ds * sin(avgThetaRad);
            currentPose.y += ds * cos(avgThetaRad);

            // Update global theta (normalized to 0-360 for display/checks if needed)
            // But keep currentPose.theta as the absolute rotation for logic if you prefer
            currentPose.theta = currentRotation; 
        }

        Pose2D getPose() {
            return currentPose;
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

        // temporary localization variables
        double prevLeftPos = 0.0;
        double prevRightPos = 0.0;
        double prevRotation = 0.0;
        double prevTime = 0.0;
        bool locInitialized = false;

        double ticksToDistance(double ticks) {
            return ticks * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / 360.0;
        }

        
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
            double headingVel = (currentHeading - previousHeading) / (dt / 1000.0);
            previousHeading = currentHeading;

            double leftVel = rawLeftVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);
            double rightVel = rawRightVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);

            double vx = 0;
            double vy = leftVel + (rightVel - leftVel) / 2;
            double w = headingVel;

            twist = {vx, vy, w};
        }
};