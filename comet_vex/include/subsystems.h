#pragma once

#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/loader.h"
#include "subsystems/arm.h"

inline Drivebase *drivebase = nullptr;
inline Intake *intake = nullptr;
inline Outtake *outtake = nullptr;
inline Loader *loader = nullptr;
inline Arm *arm = nullptr;

void subsystems_initialize(void);