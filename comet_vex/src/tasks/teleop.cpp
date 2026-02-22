#include "tasks/teleop.h"
#include "subsystems.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

void opcontrol_initialize()
{
    pros::lcd::print(1, "Initializing teleop...");

    new pros::Task([=]()
                   { intake->intakeTask(); });
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
        drivebase->signedDrive(drive, turn);

        // intake/outtake
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) // intaking from loader
        {
            loader->activate();
            intake->setIntakeMode(IntakeMode::FORWARD);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) // intaking
        {
            intake->setIntakeMode(IntakeMode::FORWARD);
            outtake->stop();
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
            loader->deactivate();
            outtake->stop();
        }

        // outtake height adjust
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            outtake->adjustUp();
        }
        else
        {
            outtake->adjustDown();
        }

        // toggle loader
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            loader->toggle();
        }

        pros::lcd::print(3, "Intake mode: %d", static_cast<int>(intake->getIntakeMode()));
        pros::lcd::print(4, "Intake Motor Temp: %f", intake->getIntakeMotorTemp());
        pros::lcd::print(5, "Outtake Extended: %d", outtake->getHeight());
        pros::lcd::print(6, "Loader Extended: %d", loader->is_extended());

        pros::delay(10);
    }
}