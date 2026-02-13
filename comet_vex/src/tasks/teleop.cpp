#include "tasks/teleop.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include <sstream>
#include "pros/llemu.hpp"

void opcontrol_initialize()
{
    pros::lcd::initialize();
}

void opcontrol()
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        double drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        drivebase->errorDrive(drive, turn);

        pros::delay(10);
    }
}