#pragma once

#include <cmath>

struct Pose2D {
    double x;
    double y;
    double theta;  // angle in radians

    Pose2D() : x(0), y(0), theta(0) {}
    
    Pose2D(double x, double y, double theta = 0)
        : x(x), y(y), theta(theta) {}

    double distance(const Pose2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Pose2D operator+(const Pose2D& other) const {
        return Pose2D(x + other.x, y + other.y, theta + other.theta);
    }

    Pose2D operator-(const Pose2D& other) const {
        return Pose2D(x - other.x, y - other.y, theta - other.theta);
    }
};