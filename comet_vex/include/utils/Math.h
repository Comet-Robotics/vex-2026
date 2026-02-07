#pragma once
#include <cmath>

inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

inline double normalizeAngleDeg(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle <= -180.0)
        angle += 360.0;
    return angle;
}

inline double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

inline double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}