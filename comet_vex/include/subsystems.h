#pragma once

#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"
#include "subsystems/loader.h"

inline Drivebase *drivebase = nullptr;
inline Intake *intake = nullptr;
inline Outtake *outtake = nullptr;
inline Loader *loader = nullptr;

void subsystems_initialize(void);