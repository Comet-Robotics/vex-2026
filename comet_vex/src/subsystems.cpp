#include "subsystems.h"

void subsystems_initialize()
{
    pros::lcd::initialize();

    drivebase = new Drivebase();
    drivebase->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    intake = new Intake();
    outtake = new Outtake();
    outtake->adjustDown();
    loader = new Loader();

    new pros::Task([=]()
                   { intake->intakeTask(); });
}