#include "main.h"
#include "tasks/auton.h"
#include "tasks/teleop.h"

void initialize()
{
	pros::lcd::initialize();

	pros::lcd::print(0, "Initializing...");

	autonomous_initialize();
	opcontrol_initialize();

	pros::lcd::print(0, "Initialization complete");
	pros::lcd::clear_line(1);
}
void disabled() {}
void competition_initialize() {}