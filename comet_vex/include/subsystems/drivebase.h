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

        void goToPoseBasic(Pose2D goal, uint32_t driveTimeout = 5000, uint32_t turnTimeout = 3000) {
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

        void goToPoseRamsete(Pose2D goal) {
            while (currentPose.distance(goal) > 1.0 || fabs(currentPose.theta - goal.theta) > 1.0) {
                double errorX = goal.x - currentPose.x;
                double errorY = goal.y - currentPose.y;
                double errorTheta = normalizeAngle(degToRad(goal.theta - currentPose.theta));

                double errorXRobot = errorX * cos(degToRad(currentPose.theta)) + errorY * sin(degToRad(currentPose.theta));
                double errorYRobot = -errorX * sin(degToRad(currentPose.theta)) + errorY * cos(degToRad(currentPose.theta));

                // calculate gain value
                const double b = 1.5;
                const double zeta = 0.7;
                const double k_v = 1.0;
                const double k_theta = 1.5;

                // TODO: replace with trajectory tracking values
                double v_d = k_v * errorXRobot;
                double w_d = k_theta * errorTheta;

                double k = 2.0 * zeta * sqrt(w_d * w_d + b * v_d * v_d);

                double sinc;
                if (fabs(errorTheta) < 1e-6) {
                    sinc = 1.0;
                }
                else {
                    sinc = sin(errorTheta) / errorTheta;
                }

                double v = v_d * cos(errorTheta) + k * errorXRobot; // target linear velocity (in/s)
                double w = w_d + k * errorTheta + (b * v_d * sinc * errorYRobot); // target angular velocity (rad/s)

                // convert to -1 to 1 range
                const double MAX_V = WHEEL_RADIUS * 2 * M_PI * DRIVETRAIN_GEAR_RATIO * 600 / 60.0; // in/s
                const double MAX_W = 2 * MAX_V / TRACK_WIDTH; // rad/s
                v = v / MAX_V; // normalize to -1 to 1
                w = w / MAX_W; // normalize to -1 to 1


                normalDrive(v, w);

                updateLocalization();

                pros::delay(20);
            }
            normalDrive(0, 0);
        }

        void goToPoseUnicycle(Pose2D goal) {
            const double k_rho = 2.5;
            const double k_alpha = 4.0;
            const double k_beta = -1.5;

            const double MAX_V = WHEEL_RADIUS * 2 * M_PI * DRIVETRAIN_GEAR_RATIO * 600 / 60.0;
            const double MAX_W = 2 * MAX_V / TRACK_WIDTH;

            while (true) {
                double theta = degToRad(currentPose.theta);
                double goalTheta = degToRad(goal.theta);

                double dx = goal.x - currentPose.x;
                double dy = goal.y - currentPose.y;

                double rho = hypot(dx, dy);
                double alpha = normalizeAngle(atan2(dy, dx) - theta);
                double beta = normalizeAngle(goalTheta - theta - alpha);

                if (rho < 1.0 && fabs(beta) < degToRad(2)) break;

                double v = k_rho * rho;
                double w = k_alpha * alpha + k_beta * beta;

                v /= MAX_V;
                w /= MAX_W;

                normalDrive(v, w);
                updateLocalization();
                pros::delay(20);
            }

            normalDrive(0,0);
        }

        double measureMaxV(uint32_t testTimeMs = 1500, bool print = true) {
            double maxV = 0;

            Pose2D prev = currentPose;

            uint32_t start = prevTime;
            uint32_t prevTime = pros::millis();
            normalDrive(1.0, 0.0);

            while (pros::millis() - start < testTimeMs) {
                pros::delay(10); // at top to allow updateLocalization to have some time

                updateLocalization();

                uint32_t now = pros::millis();
                double dt = (now - prevTime) / 1000.0;

                double dx = currentPose.x - prev.x;
                double dy = currentPose.y - prev.y;

                double v = hypot(dx, dy) / dt;

                if (v > maxV) {
                    maxV = v;
                }

                prev = currentPose;
                prevTime = now;
            }

            normalDrive(0, 0);

            if (print) {
                pros::lcd::print(0, "Measured Max V: %f in/s", maxV);
            }

            return maxV;
        }

        double measureMaxW(uint32_t testTimeMs = 1500, bool print = true) {
            double maxW = 0;

            double prevTheta = currentPose.theta;

            uint32_t start = prevTime;
            uint32_t prevTime = pros::millis();
            double prevRot = IMU.get_rotation();
            normalDrive(0.0, 1.0);

            while (pros::millis() - start < testTimeMs) {
                pros::delay(10); // at top to allow updateLocalization to have some time

                double currentRot = IMU.get_rotation();

                uint32_t now = pros::millis();
                double dt = (now - prevTime) / 1000.0;

                double dTheta = normalizeAngle(degToRad(currentRot - prevRot));

                double w = fabs(dTheta) / dt;

                if (w > maxW) {
                    maxW = w;
                }

                prevTheta = currentPose.theta;
                prevTime = now;
                prevRot = currentRot;
            }

            normalDrive(0, 0);

            if (print) {
                pros::lcd::print(0, "Measured Max W: %f rad/s", maxW);
            }

            return maxW;
        }

        void calculateMaxSpeeds(uint32_t testTimeMs = 1500) {
            pros::lcd::print(0, "Calculating max speeds...");
            pros::lcd::print(1, "Calculating max V...");
            double maxV = measureMaxV(testTimeMs, false);
            pros::delay(1000);
            pros::lcd::print(2, "Calculating max W...");
            double maxW = measureMaxW(testTimeMs, false);

            pros::lcd::print(3, "Max V: %f in/s", maxV);
            pros::lcd::print(4, "Max W: %f rad/s", maxW);
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
            double deltaHeading = currentHeading - previousHeading;
            if (deltaHeading > 180)  deltaHeading -= 360;
            if (deltaHeading < -180) deltaHeading += 360;

            double headingVel = deltaHeading * M_PI / 180.0 / (dt / 1000.0);
            previousHeading = currentHeading;

            double leftVel = rawLeftVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);
            double rightVel = rawRightVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);

            double vx = leftVel + (rightVel - leftVel) / 2;
            double vy = 0.0; // no lateral velocity in a differential drive
            double w = headingVel;

            twist = {vx, vy, w};
        }
};