#include "subsystems.h"

void subsystems_initialize()
{
    pros::lcd::print(1, "Initializing subsystems...");

    drivebase = new Drivebase();
    drivebase->calibrateChassis(true);
    drivebase->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    intake = new Intake();
    outtake = new Outtake();
    outtake->adjustDown();
    loader = new Loader();
    loader->deactivate();
}