#pragma once

#include "Trajectory.h"
#include "types/TrajectoryPoint.h"
#include "types/Pose2D.h"
#include "utils/MathUtils.h"
#include "utils/TrajectoryUtils.h"
#include "constants.h"
#include <chrono>
#include <cmath>

class TrajectoryFollower
{
public:
    using Clock = std::chrono::steady_clock;

    TrajectoryFollower(double toleranceSec = constants::autonomous::TIME_TOLERANCE)
        : tolerance(toleranceSec)
    {
        startTime = Clock::now();
    }

    void reset(const Trajectory &newTrajectory)
    {
        if (!newTrajectory.isLoaded())
        {
            throw std::runtime_error("Cannot reset TrajectoryFollower with an unloaded trajectory.");
        }
        trajectory = newTrajectory;
        startTime = Clock::now();
        finished = false;
        nextEventIndex = 0;
        started = false;
    }

    ChassisSpeeds update(const Pose2D &currentPose,
                         std::function<void(const std::string &)> onEventTriggered = nullptr)
    {
        if (!started)
        {
            startTime = Clock::now();
            started = true;
        }

        // timing
        double elapsedSec = std::chrono::duration<double>(Clock::now() - startTime).count();

        // event polling
        if (onEventTriggered)
        {
            auto &events = trajectory.getEvents();
            while (nextEventIndex < events.size() && elapsedSec >= events[nextEventIndex].time)
            {
                onEventTriggered(events[nextEventIndex].name);
                nextEventIndex++;
            }
        }

        // exit condition
        if (elapsedSec >= trajectory.getPoints().back().t)
        {
            finished = true;
            return ChassisSpeeds{0, 0, 0};
        }

        // define target point
        auto target = TrajectoryUtils::interpolate(elapsedSec, trajectory);

        pros::lcd::print(2, "Elapsed Time: %1.2f sec", elapsedSec);
        pros::lcd::print(3, "Target Point: X: %1.2f Y: %1.2f H: %1.2f", target.pose.x, target.pose.y, target.pose.heading);

        // controller update
        return controller.update(currentPose, target);
    }

    bool isFinished() const
    {
        return finished;
    }

private:
    Trajectory trajectory;
    HolonomicController controller{constants::drivetrain::X_PID, constants::drivetrain::Y_PID, constants::drivetrain::THETA_PID};
    Clock::time_point startTime;
    bool finished = false;
    double tolerance;
    size_t nextEventIndex = 0;
    bool started = false;
};
