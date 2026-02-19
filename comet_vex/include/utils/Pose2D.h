#pragma once

#include <cmath>

/**
 * @struct Pose2D
 * @brief Represents a 2D pose with x, y coordinates and a heading angle (theta)
 * This struct is used for representing the robot's position and orientation on the field. The theta angle is in radians and should be normalized to the range [-pi, pi] for consistency.
 */
struct Pose2D
{
    double x;
    double y;
    double theta; // angle in radians

    Pose2D() : x(0), y(0), theta(0) {}

    /**
     * Construct a Pose2D with given x, y, and theta (in radians)
     * @param x X coordinate in feet
     * @param y Y coordinate in feet
     * @param theta Heading angle in radians (normalized to [-pi, pi])
     */
    Pose2D(double x, double y, double theta)
        : x(x), y(y), theta(theta) {}

    /**
     * Calculate the Euclidean distance between this pose and another pose
     * @param other Another Pose2D to compare against
     * @return Distance in feet
     */
    double distance(const Pose2D &other) const
    {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Pose2D operator+(const Pose2D &other) const
    {
        return Pose2D(x + other.x, y + other.y, theta + other.theta);
    }

    Pose2D operator-(const Pose2D &other) const
    {
        return Pose2D(x - other.x, y - other.y, theta - other.theta);
    }
};