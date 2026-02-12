#pragma once

#include "constants.h"
#include "utils/Twist2D.h"
#include "pros/motor_group.hpp"
#include "utils/Pose2D.h"
#include "pros/llemu.hpp"
#include "utils/Math.h"
#include "utils/PID.h"

using namespace constants::drivebase;

/**
 * @brief Drivebase subsystem class for controlling the robot's drivetrain and localization.
 */
class Drivebase
{
    public:
        Drivebase() {
            setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
        }

        /**
         * Set the brake mode for all motors in the drivebase
          * @param mode The desired brake mode (coast, brake, or hold)
          * 
          * This function sets the brake mode for both the left and right motor groups, allowing for consistent behavior when stopping the robot. The brake mode determines how the motors behave when no power is applied: coast allows them to spin freely, brake applies resistance to slow down, and hold actively holds the position of the motors.
         */
        void setBrakeMode(pros::motor_brake_mode_e_t mode) {
            LEFT_MOTORS.set_brake_mode(mode);
            RIGHT_MOTORS.set_brake_mode(mode);
        }
        
        /**
         * Drive the robot using an "error drive" method that scales inputs based on their magnitude
          * @param drive The forward/backward input value (typically from -127 to 127)
          * @param turn The turning input value (typically from -127 to 127)
          * 
          * This method applies a non-linear scaling to the drive and turn inputs, which can help improve control at lower speeds by making the robot less sensitive to small joystick movements. The turn input is also scaled down more when the drive input is low, allowing for finer control when the robot is moving slowly. The resulting drive and turn values are then converted to motor voltages and applied to the left and right motor groups.
         */
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

        /**
         * Drive the robot using arcade drive that directly maps inputs to motor voltages
          * @param drive The forward/backward input value (typically from -127 to 127)
          * @param turn The turning input value (typically from -127 to 127)
          * 
          * This method directly maps the drive and turn inputs to motor voltages without any scaling. The drive input controls the forward and backward movement of the robot, while the turn input controls the rotation. The resulting drive and turn values are combined to calculate the voltage for the left and right motor groups, allowing for straightforward control of the robot's movement.
         */
        void arcade(float drive, float turn) {
            float max = fabs(drive) + fabs(turn);
            if (max > 1.0) {
                drive /= max;
                turn  /= max;
            }

            LEFT_MOTORS.move_voltage((drive + turn) * 12000);
            RIGHT_MOTORS.move_voltage((drive - turn) * 12000);
        }

        /**
         * Update the robot's localization based on sensor data and odometry
          * This method calculates the robot's current pose (position and orientation) by integrating sensor data from the IMU and encoders. It uses a simple kinematic model to estimate the robot's movement over time, allowing for accurate tracking of its position on the field. The calculated pose is stored in the currentPose variable, which can be used for navigation and control purposes.
         */
        void update() {
            calculateTwist();
            updateLocalization();
        }

        /**
         * Get the current twist (velocity) of the robot
          * @return A Twist2D struct containing the linear and angular velocity of the robot
          * 
          * This method returns the current twist of the robot, which includes the linear velocity in the x and y directions (vx and vy) as well as the angular velocity (w). The twist is calculated based on sensor data and can be used for control algorithms that require knowledge of the robot's velocity, such as trajectory tracking or feedback control.
         */
        Twist2D getTwist() {
            return twist;
        }

        /**
         * Calibrate the IMU sensor to determine gyro bias and ensure accurate orientation readings
          * This method performs the necessary steps to calibrate the IMU sensor, which includes resetting the sensor and measuring the gyro bias by taking multiple readings while the robot is stationary. The calculated bias is stored in the imuBias variable, which can be used to correct future gyro readings for improved accuracy in localization and control.
         */
        void calibrateIMU() {
            IMU.reset(true);
            while (IMU.is_calibrating()) {
                pros::delay(10);
            }

            // --- measure gyro bias ---
            imuBias = 0.0;
            const int samples = 200;

            for (int i = 0; i < samples; i++) {
                imuBias += IMU.get_gyro_rate().z;  // deg/s
                pros::delay(5);
            }
            imuBias /= samples;
        }

        /**
         * Set the robot's current pose (position and orientation) for localization purposes
          * @param pose A Pose2D struct containing the x, y, and theta values representing the robot's current position and heading
          * 
          * This method allows you to manually set the robot's current pose, which can be useful for initializing the localization system or correcting it if it becomes inaccurate. The pose is represented as a Pose2D struct, which includes the x and y coordinates (in inches) as well as the heading angle (theta) in degrees. Setting the current pose helps ensure that the robot's localization is accurate and can be used effectively for navigation and control.
         */
        void setPose(Pose2D pose) {
            currentPose = pose;
        }

        /**
         * Drive the robot to a specified pose (position and orientation) using a simple PID control loop
          * @param goal A Pose2D struct representing the target position and orientation for the robot to reach
          * @param driveTimeout The maximum time (in milliseconds) allowed for the driving phase of the movement (default is 5000 ms)
          * @param turnTimeout The maximum time (in milliseconds) allowed for the turning phase of the movement (default is 3000 ms)
          * 
          * This method implements a basic control loop to drive the robot to a specified pose on the field. It first turns the robot to face the target position, then drives towards it while continuously updating the robot's localization. Finally, it turns the robot to match the target orientation. The method uses PID controllers for both linear and angular control, allowing for smooth and accurate movement towards the goal pose. Timeouts are included to prevent the robot from getting stuck in case of issues with localization or control.
          * Note: This is a basic implementation and may not be suitable for all scenarios. More advanced techniques, such as trajectory tracking or model predictive control, can be implemented for improved performance.
         */
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
                arcade(0, turn);

                updateLocalization();

                pros::delay(50);
            }

            arcade(0, 0);

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

                arcade(drive, turn);

                updateLocalization();

                pros::delay(50);
            }

            arcade(0, 0);

            pros::delay(500);

            startTime = pros::millis();

            angular.reset();

            // turn to final angle
            while (fabs(currentPose.theta - goal.theta) > 0.3 && (pros::millis() - startTime) < turnTimeout) {
                double angleError = goal.theta - currentPose.theta;
                normalizeAngleDeg(angleError);

                double turn = angular.update(0, -angleError);

                arcade(0, turn);

                updateLocalization();

                pros::delay(50);
            }

            // stop motors
            arcade(0, 0);
        }

        /**
         * Drive the robot to a specified pose (position and orientation) using the Ramsete control algorithm
          * @param goal A Pose2D struct representing the target position and orientation for the robot to reach
          * 
          * This method implements the Ramsete control algorithm, which is a more advanced control technique for driving a robot to a specified pose. It calculates the necessary linear and angular velocities based on the current pose and the goal pose, allowing for smooth and accurate movement towards the target. The algorithm takes into account both the position and orientation errors, making it effective for trajectory tracking and navigation tasks. Note that this implementation assumes a simple kinematic model of the robot and may require tuning of the control gains for optimal performance.
         */
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


                arcade(v, w);

                updateLocalization();

                pros::delay(20);
            }
            arcade(0, 0);
        }

        /**
         * Drive the robot to a specified pose (position and orientation) using a unicycle model control approach
          * @param goal A Pose2D struct representing the target position and orientation for the robot to reach
          * 
          * This method implements a control approach based on the unicycle model of the robot, which calculates the necessary linear and angular velocities to drive the robot towards a specified pose. The control law is designed to ensure that the robot converges to the target position and orientation, with gains that can be tuned for desired responsiveness. The method continuously updates the robot's localization and applies the calculated velocities until the robot is within an acceptable distance and angle from the goal pose. Note that this is a basic implementation and may require tuning of the control gains for optimal performance.
         */
        void goToPoseUnicycle(Pose2D goal) {
            const double k_rho = 2.5;
            const double k_alpha = 6.0;
            const double kp_theta = 18.0;
            const double k_beta = -1.5;

            const double MAX_V = WHEEL_RADIUS * 2 * M_PI * DRIVETRAIN_GEAR_RATIO * 600 / 60.0;
            const double MAX_W = 2 * MAX_V / TRACK_WIDTH;

            double goalTheta = degToRad(goal.theta);

            bool atPosition = false;

            while (true) {
                updateLocalization();

                double theta = currentPose.theta;

                double dx = goal.x - currentPose.x;
                double dy = goal.y - currentPose.y;

                double rho = hypot(dx, dy);

                double headingToGoal = atan2(dy, dx);
                double alpha = normalizeAngle(headingToGoal - theta);
                double beta  = normalizeAngle(goalTheta - theta - alpha);

                double v, w;

                // ---- two-stage control (CRITICAL) ----
                if (!atPosition && rho < 3.0) {
                    atPosition = true;

                }
                if (atPosition) {
                    v = 0;
                    double headingError = normalizeAngle(goalTheta - theta);
                    w = kp_theta * headingError;

                    if (fabs(headingError) < degToRad(2)) {
                        break;
                    }
                }
                else {
                    // allow reversing for stability
                    if (fabs(alpha) > M_PI/2) {
                        alpha = normalizeAngle(alpha + M_PI);
                        rho = -rho;
                    }

                    v = k_rho * rho;
                    w = k_alpha * alpha + k_beta * beta;
                }

                // clamp physically
                v = std::clamp(v, -MAX_V, MAX_V);
                w = std::clamp(w, -MAX_W, MAX_W);

                arcade(v / MAX_V, w / MAX_W);

                if (rho < 3.0 && fabs(normalizeAngle(goalTheta - theta)) < degToRad(2)) {
                    break;
                }

                pros::delay(20);
            }

            arcade(0,0);
        }


        /**
         * Measure the maximum linear velocity (V) of the robot by driving at full power and tracking the distance traveled over time
          * @param testTimeMs The duration (in milliseconds) for which to run the test (default is 1500 ms)
          * @param print A boolean flag indicating whether to print the measured maximum velocity to the LCD (default is true)
          * 
          * This method drives the robot forward at full power for a specified duration while continuously updating its localization. It calculates the linear velocity based on the distance traveled and the time elapsed, keeping track of the maximum velocity achieved during the test. The measured maximum velocity can be printed to the LCD for reference, and it can be used for tuning control algorithms or understanding the robot's performance capabilities.
         */
        double measureMaxV(uint32_t testTimeMs = 1500, bool print = true) {
            double maxV = 0;

            Pose2D prev = currentPose;

            uint32_t start = prevTime;
            uint32_t prevTime = pros::millis();
            arcade(1.0, 0.0);

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

            arcade(0, 0);

            if (print) {
                pros::lcd::print(0, "Measured Max V: %f in/s", maxV);
            }

            return maxV;
        }

        /**
         * Measure the maximum angular velocity (W) of the robot by driving at full rotational power and tracking the change in orientation over time
          * @param testTimeMs The duration (in milliseconds) for which to run the test (default is 1500 ms)
          * @param print A boolean flag indicating whether to print the measured maximum angular velocity to the LCD (default is true)
          * 
          * This method rotates the robot at full power for a specified duration while continuously updating its localization. It calculates the angular velocity based on the change in orientation and the time elapsed, keeping track of the maximum angular velocity achieved during the test. The measured maximum angular velocity can be printed to the LCD for reference, and it can be used for tuning control algorithms or understanding the robot's performance capabilities in terms of rotation.
         */
        double measureMaxW(uint32_t testTimeMs = 1500, bool print = true) {
            double maxW = 0;

            double prevTheta = currentPose.theta;

            uint32_t start = prevTime;
            uint32_t prevTime = pros::millis();
            double prevRot = IMU.get_rotation();
            arcade(0.0, 1.0);

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

            arcade(0, 0);

            if (print) {
                pros::lcd::print(0, "Measured Max W: %f rad/s", maxW);
            }

            return maxW;
        }

        /**
         * Calculate the maximum linear and angular speeds of the robot by running tests and measuring the results
          * @param testTimeMs The duration (in milliseconds) for which to run each test (default is 1500 ms)
          * This method runs two separate tests to measure the maximum linear velocity (V) and maximum angular velocity (W) of the robot. It first drives the robot forward at full power to measure max V, then rotates the robot at full power to measure max W. The results are printed to the LCD for reference. These measurements can be useful for tuning control algorithms or understanding the robot's performance capabilities in terms of speed and rotation.
         */
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

        /**
         * Update the robot's localization based on sensor data and odometry
          * This method calculates the robot's current pose (position and orientation) by integrating sensor data from the IMU and encoders. It uses a simple kinematic model to estimate the robot's movement over time, allowing for accurate tracking of its position on the field. The calculated pose is stored in the currentPose variable, which can be used for navigation and control purposes.
          * The method reads the current encoder positions and IMU rotation, calculates the change in position and orientation since the last update, and updates the currentPose accordingly. It also applies a bias correction to the IMU readings to account for gyro drift over time.
         */
        void updateLocalization() {
            // --- Read sensors ---
            double leftPos  = LEFT_MOTORS.get_position();   // degrees
            double rightPos = RIGHT_MOTORS.get_position();  // degrees

            uint32_t now = pros::millis();

            // --- First-run initialization ---
            if (!locInitialized) {
                prevLeftPos = leftPos;
                prevRightPos = rightPos;
                prevRotation = IMU.get_rotation();
                prevTime = now;
                locInitialized = true;
                return;
            }

            // --- Time delta ---
            double dt = (now - prevTime) / 1000.0;
            prevTime = now;

            if (dt <= 0) return;

            // --- Encoder deltas ---
            double dLeft  = ticksToDistance(leftPos  - prevLeftPos);
            double dRight = ticksToDistance(rightPos - prevRightPos);
            prevLeftPos = leftPos;
            prevRightPos = rightPos;

            double ds = (dLeft + dRight) / 2.0;

            // --- IMU heading with bias correction ---
            double rawRotation = IMU.get_rotation(); // degrees
            double currentRotation = rawRotation - imuBias * dt;

            double prevThetaRad = degToRad(prevRotation);
            double currThetaRad = degToRad(currentRotation);
            double dTheta = normalizeAngle(currThetaRad - prevThetaRad);

            // --- Arc-based integration ---
            if (fabs(dTheta) < 1e-6) {
                // Straight motion
                currentPose.x += ds * sin(prevThetaRad);
                currentPose.y += ds * cos(prevThetaRad);
            } else {
                // Turning motion
                double r = ds / dTheta;
                currentPose.x += r * (sin(currThetaRad) - sin(prevThetaRad));
                currentPose.y += r * (cos(prevThetaRad) - cos(currThetaRad));
            }

            // --- Update pose ---
            currentPose.theta = normalizeAngle(currThetaRad);  // STORE RADIANS
            prevRotation = currentRotation;

            pros::lcd::print(4, "Pose x: %f", currentPose.x);
            pros::lcd::print(5, "Pose y: %f", currentPose.y);
            pros::lcd::print(6, "Pose theta: %f", radToDeg(currentPose.theta));
        }

        /**
         * Get the current pose (position and orientation) of the robot
          * @return A Pose2D struct containing the x, y, and theta values representing the robot's current position and heading
         */
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

        // IMU drift correction
        double imuBias = 0.0;

        /**
         * Convert encoder ticks (degrees) to linear distance traveled by the robot
          * @param ticks The number of encoder ticks (degrees) to convert
          * @return The corresponding linear distance in inches
         */
        double ticksToDistance(double ticks) {
            return ticks * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / 360.0;
        }
        
        double previousHeading = IMU.get_heading();
        double previousTime = pros::millis();
        bool pastFirstLoop = false;

        /**
         * Calculate the robot's twist (velocity and angular velocity) based on sensor data
          * This method reads the current velocities from the motor encoders and the IMU, calculates the linear and angular velocities of the robot, and stores them in the twist variable. The linear velocity is calculated based on the average of the left and right wheel velocities, while the angular velocity is derived from the change in heading over time. This information can be used for control algorithms that require knowledge of the robot's current velocity, such as trajectory tracking or feedback control.
         */
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

            double headingVel = deltaHeading / (dt / 1000.0);
            previousHeading = currentHeading;

            double leftVel = rawLeftVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);
            double rightVel = rawRightVel * 2 * M_PI * WHEEL_RADIUS * DRIVETRAIN_GEAR_RATIO / (12.0 * 60.0);

            double vx = leftVel + (rightVel - leftVel) / 2;
            double vy = 0.0; // no lateral velocity in a differential drive
            double w = headingVel;

            twist = {vx, vy, w};
        }
};