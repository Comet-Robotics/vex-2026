#pragma once

#include "subsystems/SwerveDrive.h"

inline SwerveDrive *drivebase = nullptr;

// Initialize the subsystems
inline void subsystems_initialize()
{
    drivebase = new SwerveDrive();
    constants::drivetrain::IMU.reset();
    while (constants::drivetrain::IMU.is_calibrating())
    {
        pros::delay(20);
    }
}