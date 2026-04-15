#pragma once

#include "subsystems/SwerveDrive.h"
#include "subsystems/Conveyor.h"
#include "subsystems/Blocker.h"
#include "subsystems/Loader.h"
#include "subsystems/Wings.h"
#include "subsystems/Park.h"

inline SwerveDrive *drivebase = nullptr;
inline Conveyor *conveyor = nullptr;
inline Blocker *blocker = nullptr;
inline Loader *loader = nullptr;
inline Wings *wings = nullptr;
inline Park *park = nullptr;

// Initialize the subsystems
inline void subsystems_initialize()
{
    drivebase = new SwerveDrive();
    conveyor = new Conveyor();
    blocker = new Blocker();
    loader = new Loader();
    wings = new Wings();
    park = new Park();
    drivebase->imu->reset();
    while (constants::drivetrain::IMU.is_calibrating())
    {
        pros::delay(20);
    }
}