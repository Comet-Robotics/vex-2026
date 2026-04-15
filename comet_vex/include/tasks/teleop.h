#pragma once

#ifdef __cplusplus
#include "pros/misc.hpp"
extern pros::Controller controller;
void drivebase_controls();
extern "C" {
#endif

    void opcontrol(void);
    void opcontrol_initialize(void);

#ifdef __cplusplus
}
#endif