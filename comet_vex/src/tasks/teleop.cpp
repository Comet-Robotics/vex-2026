#include "tasks/teleop.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void opcontrol_initialize()
{
}

void opcontrol()
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    drivebase->setPoseComet(0, 0, 0);

    while (true)
    {
        bool loaderDeployed = false;

        // drivebase
        double drive = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        drivebase->signedDrive(drive, turn);

        // intake/outtake
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) // intaking from loader
        {
            loaderDeployed = true;
            conveyor->forward();
            blocker->activate();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) // intaking
        {
            conveyor->forward();
            blocker->activate();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // outtaking
        {
            conveyor->reverse();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) // scoring
        {
            if (!conveyor->isHigh())
            {
                conveyor->forwardSlow();
            }
            else
            {
                conveyor->forward();
            }
            blocker->deactivate();
        }
        else // stop
        {
            conveyor->stop();
            loaderDeployed = false;
        }

        // outtake height adjust
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            conveyor->adjustUp();
        }
        else
        {
            conveyor->adjustDown();
        }

        // deploy loader
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            loaderDeployed = true;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            loaderDeployed = false;
        }

        // double pitch = drivebase->getIMU().get_pitch();

        // // anti-tip loader deploy
        // // only applies if robot is tipping forward
        // if (pitch < PITCH_THRESHOLD)
        // {
        //     loaderDeployed = true;
        // }

        // set loader state
        if (loaderDeployed)
        {
            loader->activate();
        }
        else
        {
            loader->deactivate();
        }

        // toggle loader
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //     loader->toggle();
        // }

        // IntakeMode currentIntakeMode = intake->getIntakeMode();
        // pros::lcd::print(0, "Intake Mode: %s", (currentIntakeMode == IntakeMode::OFF) ? "OFF" : (currentIntakeMode == IntakeMode::FORWARD) ? "FORWARD"
        //                                                                                     : (currentIntakeMode == IntakeMode::REVERSE)   ? "REVERSE"
        //                                                                                     : (currentIntakeMode == IntakeMode::UNFOLD)    ? "UNFOLD"
        //
        // pros::lcd::print(0, "Outtake Height: %s", (outtake->isHigh() ? "HIGH" : "LOW"));
        // pros::lcd::print(1, "Loader: %s", loaderDeployed ? "DEPLOYED" : "RETRACTED");
        // pros::lcd::print(2, "Tipping Forward: %s", (pitch < PITCH_THRESHOLD) ? "YES" : "NO");
        // pros::lcd::print(3, "Pitch: %f", pitch);

        // pros::lcd::print(1, "Pose: (%f, %f, %f)", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().theta);
        pros::delay(20);
    }
}