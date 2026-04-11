#pragma once
#include "types/TrajectoryPoint.h"
#include "types/Pose2D.h"
#include "motion/Trajectory.h"

namespace TrajectoryUtils
{
    inline TrajectoryPoint interpolate(double t, Trajectory &trajectory)
    {
        const auto &points = trajectory.getPoints();
        if (points.empty())
            return TrajectoryPoint{};

        if (t <= points.front().t)
            return points.front();
        if (t >= points.back().t)
            return points.back();

        for (size_t i = 1; i < points.size(); ++i)
        {
            const auto &prev = points[i - 1];
            const auto &next = points[i];
            if (t < next.t)
            {
                double ratio = (t - prev.t) / (next.t - prev.t);
                return TrajectoryPoint{
                    {std::lerp(prev.pose.x, next.pose.x, ratio),
                     std::lerp(prev.pose.y, next.pose.y, ratio),
                     AngleUtils::lerpAngle(prev.pose.heading, next.pose.heading, ratio, false)},
                    {std::lerp(prev.velocity.x, next.velocity.x, ratio),
                     std::lerp(prev.velocity.y, next.velocity.y, ratio),
                     std::lerp(prev.velocity.heading, next.velocity.heading, ratio)},
                    t};
            }
        }

        return points.back();
    }
}