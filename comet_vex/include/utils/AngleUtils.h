#pragma once

#include <cmath>

namespace AngleUtils
{

    inline double wrap2Pi(double angle)
    {
        angle = fmod(angle, 2 * M_PI);
        if (angle < 0)
            angle += 2 * M_PI;
        return angle;
    }

    inline double wrapPi(double angle)
    {
        angle = fmod(angle + M_PI, 2 * M_PI);
        if (angle < 0)
            angle += 2 * M_PI;
        return angle - M_PI;
    }

    // Wraps an angle to the range [0, 360)
    inline double wrap360(double angle)
    {
        angle = fmod(angle, 360.0);
        if (angle < 0)
            angle += 360.0;
        return angle;
    }

    // Wraps an angle to the range (-180, 180]
    inline double wrap180(double angle)
    {
        angle = fmod(angle + 180.0, 360.0);
        if (angle < 0)
            angle += 360.0;
        return angle - 180.0;
    }

    inline double shortestAngleDelta(double from, double to, bool inDegrees = true)
    {
        double delta = to - from;
        if (inDegrees)
        {
            delta = wrap180(delta);
        }
        else
        {
            delta = wrapPi(delta);
        }
        return delta;
    }

    // Returns optimized angle and speed based on delta (minimize rotation)
    inline void optimizeAngleAndSpeed(double currentAngle, double &targetAngle, double &targetSpeed)
    {
        currentAngle = wrap360(currentAngle);
        targetAngle = wrap360(targetAngle);

        double delta = shortestAngleDelta(currentAngle, targetAngle);

        if (std::abs(delta) > 90.0)
        {
            targetSpeed *= -1;
            targetAngle = wrap360(targetAngle + 180.0);
        }
    }

    inline double toRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    inline double toDegrees(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    inline double lerpAngle(double a, double b, double t, bool inDegrees = true)
    {
        double delta = shortestAngleDelta(a, b, inDegrees);
        if (inDegrees) {
            return wrap360(a + t * delta);
        } else {
            return wrap2Pi(a + t * delta);
        }
    }

}
