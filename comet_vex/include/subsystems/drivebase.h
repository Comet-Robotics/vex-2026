#pragma once

#include "constants.h"
#include "utils/Twist2D.h"
#include "pros/motor_group.hpp"
#include "utils/Pose2D.h"
#include "pros/llemu.hpp"

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

        void normalDrive(float drive, float turn) {
            // float max = abs(drive) + abs(turn);
            // if (max > 1.0) {
            //     drive /= max;
            //     turn  /= max;
            // }

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
            double kPLinear = 0.5;
            double kPAngular = 0.01;
            
            uint32_t startTime = pros::millis();

            // drive to position
            while (currentPose.distance(goal) > 1.0 && (pros::millis() - startTime) < driveTimeout) {
                Pose2D error = goal - currentPose;
                
                pros::lcd::print(1, "error x: %f", error.x);
                pros::lcd::print(2, "error y: %f", error.y);
                pros::lcd::print(3, "error theta: %f", error.theta);
                pros::lcd::print(4, "current theta: %f", currentPose.theta);

                // project onto robot heading
                double driveError = error.x * cos(currentPose.theta) + error.y * sin(currentPose.theta);

                double angleError = atan2(error.y, error.x) - currentPose.theta;
                while(angleError > M_PI) angleError -= 2*M_PI;
                while(angleError < -M_PI) angleError += 2*M_PI;

                double drive = kPLinear * -driveError;
                double turn  = kPAngular * angleError;

                normalDrive(drive, turn);

                updateLocalization();

                pros::delay(10);
            }

            startTime = pros::millis();

            // turn to final angle
            while (fabs(currentPose.theta - goal.theta) > 0.1 && (pros::millis() - startTime) < turnTimeout) {
                double angleError = goal.theta - currentPose.theta;
                while(angleError > M_PI) angleError -= 2*M_PI;
                while(angleError < -M_PI) angleError += 2*M_PI;

                double turn  = kPAngular * angleError;

                normalDrive(0, turn);

                updateLocalization();

                pros::delay(10);
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
            
            currentPose.x += ds * cos(avgThetaRad);
            currentPose.y += ds * sin(avgThetaRad);

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