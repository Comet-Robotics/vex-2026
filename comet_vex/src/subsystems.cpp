#include "subsystems.h"
#include "pros/llemu.hpp"

void subsystems_initialize()
{
    pros::lcd::initialize();

    drivebase = new Drivebase();
    drivebase->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    drivebase->calibrateChassis(true);

    conveyor = new Conveyor();
    conveyor->adjustDown();
    loader = new Loader();
    blocker = new Blocker();
}