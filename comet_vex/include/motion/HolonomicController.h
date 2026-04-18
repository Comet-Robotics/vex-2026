#pragma once

#include "utils/AngleUtils.h"
#include "utils/PID.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <tuple>
#include "types/Pose2D.h"
#include "types/TrajectoryPoint.h"
#include "types/ChassisSpeeds.h"
#include "utils/AngleUtils.h"

class HolonomicController
{
public:
    HolonomicController(std::array<double, 3> xPIDCoefficients,
                        std::array<double, 3> yPIDCoefficients,
                        std::array<double, 3> thetaPIDCoefficients)
        : xPID(xPIDCoefficients[0], xPIDCoefficients[1], xPIDCoefficients[2]),
          yPID(yPIDCoefficients[0], yPIDCoefficients[1], yPIDCoefficients[2]),
          thetaPID(thetaPIDCoefficients[0], thetaPIDCoefficients[1], thetaPIDCoefficients[2]) {}

    ChassisSpeeds update(const Pose2D &currentPose, const TrajectoryPoint &target)
    {
        Pose2D targetPose = target.pose;
        Pose2D targetVelocity = target.velocity;

        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;

        double dtheta = AngleUtils::shortestAngleDelta(targetPose.heading, currentPose.heading, false);

        double correctionX = xPID.calculate(dx);
        double correctionY = yPID.calculate(dy);
        double correctionTheta = thetaPID.calculate(dtheta);

        // double correctionX = 0;
        // double correctionY = 0;
        // double correctionTheta = 0;

        double forward = targetVelocity.x + correctionX;
        double strafe = targetVelocity.y + correctionY;
        double rotation = targetVelocity.heading + correctionTheta;

        logError(currentPose, targetPose);

        return ChassisSpeeds{forward, -strafe, AngleUtils::toDegrees(rotation)};
    }

    ChassisSpeeds updateTest(const Pose2D &currentPose, const Pose2D &targetPose)
    {
        double dx = targetPose.x - currentPose.x;
        double dy = targetPose.y - currentPose.y;
        double dtheta = AngleUtils::shortestAngleDelta(targetPose.heading, currentPose.heading, false);

        double correctionX = xPID.calculate(dx);
        double correctionY = yPID.calculate(dy);
        double correctionTheta = thetaPID.calculate(dtheta);

        // double sinHeading = sin(currentPose.heading);
        // double cosHeading = cos(currentPose.heading);
        // double correctionXLocal = correctionX * cosHeading + correctionY * sinHeading;
        // double correctionYLocal = correctionX * sinHeading - correctionY * cosHeading;

        // pros::lcd::print(x, "Error: X: %1.2f Y: %1.2f H: %1.2f", dx, dy, dtheta);
        // pros::lcd::print(x, "Correction: X: %1.2f Y: %1.2f H: %1.2f", correctionX, correctionY, correctionTheta);
        // // pros::lcd::print(x, "Correction Local: X: %1.2f Y: %1.2f", correctionXLocal, correctionYLocal);

        // return ChassisSpeeds{correctionXLocal, correctionYLocal, correctionTheta};
        return ChassisSpeeds{correctionX, -correctionY, AngleUtils::toDegrees(correctionTheta)};
    }

private:
    PID xPID, yPID, thetaPID;

    void logError(const Pose2D &currentPose, const Pose2D &targetPose)
    {
        printf("[Tracking] ΔX: %.2f, ΔY: %.2f, Δθ: %.2f\n",
               targetPose.x - currentPose.x,
               targetPose.y - currentPose.y,
               AngleUtils::shortestAngleDelta(targetPose.heading, currentPose.heading, false));
    }
};
