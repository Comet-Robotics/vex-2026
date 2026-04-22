#pragma once

#include "constants.h"
#include "utils/AngleUtils.h"
#include "utils/MathUtils.h"

#include "SwerveModule.h"
#include <cmath>
#include "motion/HolonomicController.h"
#include "motion/TrajectoryFollower.h"
#include "motion/odometry/Odometry.h"

#define EIGEN_DONT_VECTORIZE
#include "Eigen/Dense"

// #include "pros/llemu.hpp"

using namespace constants::drivetrain;
using namespace constants::ports;
using namespace Eigen;

class SwerveDrive
{
public:
    SwerveModule frontRight, frontLeft, backLeft, backRight;
    pros::Imu *imu = &constants::drivetrain::IMU;

    int count = 0;

    SwerveDrive() : frontRight(FRONT_RIGHT_PORTS[0], FRONT_RIGHT_PORTS[1], FRONT_RIGHT_ROTATION_SENSOR_PORT),
                    frontLeft(FRONT_LEFT_PORTS[0], FRONT_LEFT_PORTS[1], FRONT_LEFT_ROTATION_SENSOR_PORT),
                    backLeft(BACK_LEFT_PORTS[0], BACK_LEFT_PORTS[1], BACK_LEFT_ROTATION_SENSOR_PORT),
                    backRight(BACK_RIGHT_PORTS[0], BACK_RIGHT_PORTS[1], BACK_RIGHT_ROTATION_SENSOR_PORT)
    {
        frontRight.setPID(FRONT_RIGHT_PID);
        frontLeft.setPID(FRONT_LEFT_PID);
        backLeft.setPID(BACK_LEFT_PID);
        backRight.setPID(BACK_RIGHT_PID);

        setPose({0, 0, 0});
    }

    void setModuleSpeeds(double forward, double strafe, double rotation, bool fieldCentric = true)
    {
        if (constants::drivetrain::HEADING_HOLD && headingHoldEnabled)
        {
            double currentHeading = getHeading();
            double currentHeadingClockwise = AngleUtils::wrap360(-currentHeading);

            if (std::abs(rotation) > DEADZONE_THRESHOLD * constants::drivetrain::MAX_ANGULAR_SPEED)
            {
                isHeadingHoldActive = false;
            }
            else
            {
                // when rotation stops, set target heading to current heading
                // if the robot is still spinning fast from momentum, keep updating it
                // so we don't snap back to an old heading
                double angularVelocity = std::abs(imu->get_gyro_rate().z);

                // pros::lcd::print(x, "AngVelo: %1.2f", angularVelocity);

                if (!isHeadingHoldActive)
                {
                    targetHeading = currentHeadingClockwise;

                    // only lock in the heading once the robot's momentum has settled
                    if (angularVelocity <= 5.0)
                    {
                        isHeadingHoldActive = true;
                    }

                    // coast while waiting to settle
                    rotation = 0.0;
                }
                else
                {
                    double error = AngleUtils::shortestAngleDelta(currentHeadingClockwise, targetHeading);
                    rotation = headingHold.calculate(error) * constants::drivetrain::MAX_ANGULAR_SPEED;
                }
            }
        }

        if (fieldCentric)
        {
            double angle = AngleUtils::toRadians(-getHeading());
            double newStrafe = strafe * cos(angle) - forward * sin(angle);
            double newForward = forward * cos(angle) + strafe * sin(angle);
            strafe = newStrafe;
            forward = newForward;
        }

        // This is the original method - works, but forward kinematics will be easier with matrices

        // double a = strafe - rotation;
        // double b = strafe + rotation;
        // double c = forward - rotation;
        // double d = forward + rotation;

        // double frontRightSpeed = hypot(b, c);
        // double frontRightAngle = AngleUtils::toDegrees(atan2(b, c));
        // double frontLeftSpeed = hypot(b, d);
        // double frontLeftAngle = AngleUtils::toDegrees(atan2(b, d));
        // double backLeftSpeed = hypot(a, d);
        // double backLeftAngle = AngleUtils::toDegrees(atan2(a, d));
        // double backRightSpeed = hypot(a, c);
        // double backRightAngle = AngleUtils::toDegrees(atan2(a, c));

        // // pros::lcd::print(x, "FR: %1.2f @ %1.2f, FL: %1.2f @ %1.2f",
        //                  frontRightSpeed, frontRightAngle,
        //                  frontLeftSpeed, frontLeftAngle);
        // // pros::lcd::print(x, "BL: %1.2f @ %1.2f, BR: %1.2f @ %1.2f",
        //                  backLeftSpeed, backLeftAngle,
        //                  backRightSpeed, backRightAngle);

        // frontRight.setSpeedAndAngle(frontRightSpeed, frontRightAngle);
        // frontLeft.setSpeedAndAngle(frontLeftSpeed, frontLeftAngle);
        // backLeft.setSpeedAndAngle(backLeftSpeed, backLeftAngle);
        // backRight.setSpeedAndAngle(backRightSpeed, backRightAngle);

        MatrixXd wheelVectors = CONVERSION_MATRIX * Vector3d(forward, -strafe, -AngleUtils::toRadians(rotation));

        MatrixXd wheelStates(4, 2);
        double maxCalculatedSpeed = 0;
        for (int i = 0; i < 4; i++)
        {
            double wheelSpeed = sqrt(pow(wheelVectors(i * 2), 2) + pow(wheelVectors(i * 2 + 1), 2));
            double wheelAngle;

            if (constants::drivetrain::STAY_AT_ORIENTATION && wheelSpeed < DEADZONE_THRESHOLD * constants::drivetrain::MAX_LINEAR_SPEED)
            {
                wheelAngle = getModule(i).getState()[1]; // if the wheel isn't moving, just keep the same angle
            }
            else
            {
                wheelAngle = AngleUtils::toDegrees(atan2(-wheelVectors(i * 2 + 1), wheelVectors(i * 2)));
            }

            wheelStates(i, 0) = wheelSpeed;
            wheelStates(i, 1) = wheelAngle;

            if (wheelSpeed > maxCalculatedSpeed)
            {
                maxCalculatedSpeed = wheelSpeed;
            }
        }

        double MAX_SPEED = constants::drivetrain::MAX_LINEAR_SPEED;

        if (maxCalculatedSpeed > MAX_SPEED)
        {
            for (int i = 0; i < 4; i++)
            {
                // scale speed down
                wheelStates(i, 0) = (wheelStates(i, 0) / maxCalculatedSpeed) * MAX_SPEED;
            }
        }

        frontRight.setSpeedAndAngle(wheelStates(0, 0), wheelStates(0, 1));
        frontLeft.setSpeedAndAngle(wheelStates(1, 0), wheelStates(1, 1));
        backLeft.setSpeedAndAngle(wheelStates(2, 0), wheelStates(2, 1));
        backRight.setSpeedAndAngle(wheelStates(3, 0), wheelStates(3, 1));
        if (count % 10 == 0)
        {
            printf("Swerve Angles: FR: %1.2f, FL: %1.2f, BL: %1.2f, BR: %1.2f\n",
                   wheelStates(0, 1), wheelStates(1, 1), wheelStates(2, 1), wheelStates(3, 1));
            overCurrentDetector();
        }
        count++;
    }

    void basicSetModuleSpeeds(double forward, double strafe, double rotation, bool fieldCentric = true)
    {
        if (fieldCentric)
        {
            double angle = AngleUtils::toRadians(-getHeading());
            double newStrafe = strafe * cos(angle) - forward * sin(angle);
            double newForward = forward * cos(angle) + strafe * sin(angle);
            strafe = newStrafe;
            forward = newForward;
        }

        MatrixXd wheelVectors = CONVERSION_MATRIX * Vector3d(forward, -strafe, -AngleUtils::toRadians(rotation));

        MatrixXd wheelStates(4, 2);
        double maxCalculatedSpeed = 0;

        for (int i = 0; i < 4; i++)
        {
            double wheelSpeed = sqrt(pow(wheelVectors(i * 2), 2) + pow(wheelVectors(i * 2 + 1), 2));
            double wheelAngle = AngleUtils::toDegrees(atan2(-wheelVectors(i * 2 + 1), wheelVectors(i * 2)));

            wheelStates(i, 0) = wheelSpeed;
            wheelStates(i, 1) = wheelAngle;

            if (wheelSpeed > maxCalculatedSpeed)
            {
                maxCalculatedSpeed = wheelSpeed;
            }
        }

        double MAX_SPEED = constants::drivetrain::MAX_LINEAR_SPEED;

        if (maxCalculatedSpeed > MAX_SPEED)
        {
            for (int i = 0; i < 4; i++)
            {
                // scale speed down
                wheelStates(i, 0) = (wheelStates(i, 0) / maxCalculatedSpeed) * MAX_SPEED;
            }
        }

        frontRight.setSpeedAndAngle(wheelStates(0, 0), wheelStates(0, 1));
        frontLeft.setSpeedAndAngle(wheelStates(1, 0), wheelStates(1, 1));
        backLeft.setSpeedAndAngle(wheelStates(2, 0), wheelStates(2, 1));
        backRight.setSpeedAndAngle(wheelStates(3, 0), wheelStates(3, 1));
    }

    /**
     * Set the wheels to an X formation to resist being pushed around.
     */
    void xWheels()
    {
        frontRight.setSpeedAndAngle(0, X_ANGLE);
        frontLeft.setSpeedAndAngle(0, -X_ANGLE);
        backLeft.setSpeedAndAngle(0, X_ANGLE);
        backRight.setSpeedAndAngle(0, -X_ANGLE);
    }

    void update()
    {
        frontRight.update();
        frontLeft.update();
        backLeft.update();
        backRight.update();

        odometry.update(frontRight, frontLeft, backLeft, backRight, AngleUtils::toRadians(getHeading()));
        // printf("[SwerveDrive] Pose - X: %.2f, Y: %.2f, Heading: %.2f\n", currentPose.x, currentPose.y, currentPose.heading);
    }

    // void setAutonomous(bool autonomous) {
    //     this->autonomous = autonomous;
    // }

    void setPose(const Pose2D &pose)
    {
        odometry.setPose(pose);
        imu->set_heading(AngleUtils::toDegrees(AngleUtils::wrap2Pi(-pose.heading)));
    }

    void setY(double y)
    {
        Pose2D pose = getPose();
        pose.y = y;
        setPose(pose);
    }

    double getDistanceOffset()
    {
        double distVal = getDistance();
        double heading = getHeading();
        double dist = distVal * -cos(AngleUtils::toRadians(heading));
        double sensor_offset = (distOffsetX * sin(AngleUtils::toRadians(heading)) + distOffsetY * -cos(AngleUtils::toRadians(heading)));
        return wallY - sensor_offset - dist;
    }

    double getDistance()
    {
        return MathUtils::metersToInches(DISTANCE.get_distance()) / 1000.0;
    }

    Pose2D getPose()
    {
        return odometry.getPose();
    }

    void resetPose()
    {
        odometry.reset();
        tareIMU();
    }

    void tareIMU(bool calibrate = false)
    {
        if (calibrate)
        {
            imu->reset(true);
        }
        imu->tare();
        targetHeading = 0;
    }

    void setTrajectory(const Trajectory &trajectory)
    {
        follower.reset(trajectory);
        setPose(trajectory.getStart().pose);
    }

    void setTrajectory(const std::string &filename)
    {
        try
        {
            Trajectory trajectory(filename);
            setTrajectory(trajectory);
        }
        catch (const std::exception &e)
        {
            printf("Failed to load trajectory: %s\n", e.what());
        }
    }

    bool atEnd() const
    {
        return follower.isFinished();
    }

    SwerveModule &getModule(int index)
    {
        switch (index)
        {
        case 0:
            return frontRight;
        case 1:
            return frontLeft;
        case 2:
            return backLeft;
        case 3:
            return backRight;
        default:
            throw std::out_of_range("Invalid module index");
        }
    }

    void toggleHeadingHold()
    {
        headingHoldEnabled = !headingHoldEnabled;
        if (!headingHoldEnabled)
        {
            isHeadingHoldActive = false;
        }
    }

    void testHolonomicController()
    {
        Pose2D targetPose{24, 24, AngleUtils::toRadians(-90)};
        headingHoldEnabled = false;

        while (true)
        {
            Pose2D currentPose = getPose();
            ChassisSpeeds speeds = controller.updateTest(currentPose, targetPose);
            basicSetModuleSpeeds(speeds.vx, speeds.vy, speeds.omega);
            update();

            // pros::lcd::print(x, "Current: X: %1.2f Y: %1.2f H: %1.2f", currentPose.x, currentPose.y, currentPose.heading);
            // pros::lcd::print(x, "Target: X: %1.2f Y: %1.2f H: %1.2f", targetPose.x, targetPose.y, targetPose.heading);
            pros::delay(20);
        }
    }

    void goToPose(const Pose2D &targetPose)
    {
        headingHoldEnabled = false;

        int32_t startTime = pros::millis();

        while (true)
        {
            Pose2D currentPose = getPose();
            ChassisSpeeds speeds = controller.updateTest(currentPose, targetPose);
            basicSetModuleSpeeds(speeds.vx, speeds.vy, speeds.omega);
            update();

            // pros::lcd::print(x, "Current: X: %1.2f Y: %1.2f H: %1.2f", currentPose.x, currentPose.y, currentPose.heading);
            // pros::lcd::print(x, "Target: X: %1.2f Y: %1.2f H: %1.2f", targetPose.x, targetPose.y, targetPose.heading);

            bool positionReached = currentPose.distance(targetPose) < 0.5;
            bool headingReached = std::abs(AngleUtils::shortestAngleDelta(currentPose.heading, targetPose.heading, false)) < AngleUtils::toRadians(1);
            bool timeout = pros::millis() - startTime > 2500;

            if ((positionReached && headingReached) || timeout)
            {
                if (timeout)
                {
                    printf("goToPose timed out\n");
                }
                else
                {
                    printf("Target pose reached\n");
                }
                break;
            }

            pros::delay(20);
        }
    }

    bool isOverTemperature() const
    {
        return frontRight.isOverTemperature() || frontLeft.isOverTemperature() || backLeft.isOverTemperature() || backRight.isOverTemperature();
    }

    void overCurrentDetector() const
    {
        frontRight.overCurrentDetector();
        frontLeft.overCurrentDetector();
        backLeft.overCurrentDetector();
        backRight.overCurrentDetector();
    }

    double getHeading()
    {
        return -imu->get_heading(); // IMU rotation is clockwise positive, we want counterclockwise positive
    }

private:
    // bool autonomous = false;
    bool isHeadingHoldActive = false;
    bool headingHoldEnabled = constants::drivetrain::HEADING_HOLD;
    double targetHeading = 0;

    Odometry odometry;

    TrajectoryFollower follower{constants::autonomous::TIME_TOLERANCE};
    HolonomicController controller{X_PID, Y_PID, THETA_PID};
    PID headingHold{HEADING_HOLD_PID[0], HEADING_HOLD_PID[1], HEADING_HOLD_PID[2]};

    // intuition says it's the opposite, but check math on atan2 to see why it's this
    double X_ANGLE = AngleUtils::toDegrees(atan2(TRACK_WIDTH, TRACK_LENGTH));

    Pose2D currentPose;
};