#pragma once

#include "pros/rtos.hpp"
#include "types/Pose2D.h"
#include "utils/AngleUtils.h"
#include <cmath>
#include <sys/types.h>
#include "subsystems/SwerveModule.h"

#define EIGEN_DONT_VECTORIZE
#include "Eigen/Dense"

using namespace Eigen;
using namespace constants::drivetrain;

class Odometry
{
public:
    Odometry() {}

    void update(const SwerveModule &frontRight, const SwerveModule &frontLeft, const SwerveModule &backLeft, const SwerveModule &backRight, double heading)
    {
        // create 8x1 matrix of wheel velocity components (vx, vy for each wheel)
        // get wheel states (speed, angle) for each wheel
        VectorXd wheelStates(8);
        auto frState = frontRight.getState();
        auto flState = frontLeft.getState();
        auto blState = backLeft.getState();
        auto brState = backRight.getState();

        // convert from polar coordinates (speed, angle) to cartesian coordinates (vx, vy)
        // Note: angles follow the convention where 0° is forward, and positive angles go clockwise
        // This matches the atan2(x_component, -y_component) convention used in SwerveDrive
        wheelStates << frState[0] * sin(AngleUtils::toRadians(frState[1])), // FR vx (forward)
            -frState[0] * cos(AngleUtils::toRadians(frState[1])),           // FR vy (left positive)
            flState[0] * sin(AngleUtils::toRadians(flState[1])),            // FL vx (forward)
            -flState[0] * cos(AngleUtils::toRadians(flState[1])),           // FL vy (left positive)
            blState[0] * sin(AngleUtils::toRadians(blState[1])),            // BL vx (forward)
            -blState[0] * cos(AngleUtils::toRadians(blState[1])),           // BL vy (left positive)
            brState[0] * sin(AngleUtils::toRadians(brState[1])),            // BR vx (forward)
            -brState[0] * cos(AngleUtils::toRadians(brState[1]));           // BR vy (left positive)

        // calculate robot velocities
        VectorXd velocities = CONVERSION_MATRIX.completeOrthogonalDecomposition().pseudoInverse() * wheelStates;

        // get change in time
        u_int32_t currentTime = pros::millis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        // if dt is too large, ignore update to prevent huge jumps in position from accumulated error while sitting idle
        if (dt > 0.5)
        {
            printf("Large dt detected (%.2f seconds), skipping odometry update to prevent position jump\n", dt);
            return;
        }

        // convert robot-centric velocities to field-centric and integrate to get new pose
        // velocities(0) is forward velocity, velocities(1) is strafe velocity (left = positive for some reason), velocities(2) is angular velocity

        currentPose.x += (velocities(0) * sin(currentPose.heading) - velocities(1) * cos(currentPose.heading)) * dt;
        currentPose.y += (-velocities(0) * cos(currentPose.heading) - velocities(1) * sin(currentPose.heading)) * dt;
        currentPose.heading = AngleUtils::wrap2Pi(heading);

        // printf("[Odometry] X: %.2f, Y: %.2f, Heading: %.2f\n", currentPose.x, currentPose.y, currentPose.heading);
    }

    void setPose(const Pose2D &pose)
    {
        currentPose = pose;
    }

    void reset()
    {
        setPose(Pose2D{0, 0, 0});
        lastUpdateTime = pros::millis();
    }

    Pose2D getPose() const
    {
        return currentPose;
    }

private:
    Pose2D currentPose;
    u_int32_t lastUpdateTime = pros::millis();
};