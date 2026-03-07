#pragma once

#include "subsystems/drivebase.h"
#include "subsystems/conveyor.h"
#include "subsystems/loader.h"
#include "subsystems/blocker.h"

inline Drivebase *drivebase = nullptr;
inline Conveyor *conveyor = nullptr;
inline Loader *loader = nullptr;
inline Blocker *blocker = nullptr;

void subsystems_initialize(void);