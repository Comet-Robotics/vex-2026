#pragma once

#include "subsystems/drivebase.h"
#include "subsystems/intake.h"
#include "subsystems/outtake.h"

inline Drivebase* drivebase = nullptr;
inline Intake* intake = nullptr;
inline Outtake* outtake = nullptr;

void subsystems_initialize(void);