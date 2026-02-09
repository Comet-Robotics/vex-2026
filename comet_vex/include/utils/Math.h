#pragma once
#include <cmath>

/**
 * @file Math.h
 * @brief Utility functions for mathematical operations commonly used in robotics, such as angle normalization and unit conversions.
 * This header provides functions to normalize angles to standard ranges (e.g., [-pi, pi] for radians and [-180, 180] for degrees) and to convert between degrees and radians. These functions are essential for ensuring consistent angle representations throughout the codebase, especially when working with robot localization and control algorithms.
 */

 /**
  * Normalize an angle in radians to the range [-pi, pi]
  * @param angle The angle in radians to normalize
  * @return The normalized angle in radians, within the range [-pi, pi]
  */
inline double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle <= -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

/**
 * Normalize an angle in degrees to the range [-180, 180]
 * @param angle The angle in degrees to normalize
 * @return The normalized angle in degrees, within the range [-180, 180]
 */
inline double normalizeAngleDeg(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle <= -180.0)
        angle += 360.0;
    return angle;
}

/**
 * Convert an angle from degrees to radians
 * @param degrees The angle in degrees to convert
 * @return The angle converted to radians
 */
inline double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

/**
 * Convert an angle from radians to degrees
 * @param radians The angle in radians to convert
 * @return The angle converted to degrees
 */
inline double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}