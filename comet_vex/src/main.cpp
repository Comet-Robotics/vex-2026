#include "main.h"
#include "liblvgl/llemu.hpp"
#include "subsystems.h"
#include "tasks/auton.h"
#include "tasks/teleop.h"

void initialize()
{
	pros::lcd::initialize();

	autonomous_initialize();
	opcontrol_initialize();
	subsystems_initialize();
}

void disabled() {}

void competition_initialize() {}
