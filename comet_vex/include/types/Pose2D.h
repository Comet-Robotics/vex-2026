#pragma once

struct Pose2D
{
    double x;
    double y;
    double heading;

    double distance(const Pose2D &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};