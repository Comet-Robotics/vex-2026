#include "main.h"
#include "liblvgl/llemu.hpp"
#include "subsystems.h"
#include "tasks/auton.h"
#include "tasks/teleop.h"

void initialize() {
	pros::lcd::initialize();

	subsystems_initialize();
	autonomous_initialize();
	opcontrol_initialize();
}
void disabled() {}
void competition_initialize() {}