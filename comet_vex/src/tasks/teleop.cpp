#include "tasks/teleop.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include <sstream>
#include "pros/llemu.hpp"

void opcontrol_initialize()
{
    pros::lcd::initialize();

    new pros::Task([=]() { intake->intakeTask(); });
}

void opcontrol()
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    intake->setIntakeMode(IntakeMode::UNFOLD);

    while (true)
    {
        // drivebase
        double drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        drivebase->errorDrive(drive, turn);

        // intake/outtake
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) // intaking
        {
            intake->setIntakeMode(IntakeMode::FORWARD);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // outtaking
        {
            intake->setIntakeMode(IntakeMode::REVERSE);
            outtake->reverse();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) // scoring
        {
            intake->setIntakeMode(IntakeMode::FORWARD);
            outtake->forward();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) // fallback to unfold intake
        {
            intake->setIntakeMode(IntakeMode::UNFOLD);
        }
        else // stop
        {
            intake->setIntakeMode(IntakeMode::OFF);
            outtake->stop();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // TODO: outtake height adjust
        }

        // toggle loader
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //     loader->toggle();
        // }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            loader->activate();
            intake->setIntakeMode(IntakeMode::FORWARD);
        }

        pros::delay(10);
    }
}