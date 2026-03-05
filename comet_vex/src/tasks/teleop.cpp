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

    intake->setIntakeMode(IntakeMode::UNFOLD);

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
            if (!outtake->isHigh())
            {
                outtake->forwardSlow();
            }
            else
            {
                outtake->forward();
            }
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) // fallback to unfold intake
        {
            intake->setIntakeMode(IntakeMode::UNFOLD);
        }
        else // stop
        {
            intake->setIntakeMode(IntakeMode::OFF);
            loaderDeployed = false;
            outtake->stop();
        }

        // outtake height adjust
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            outtake->adjustDown();
            loaderDeployed = true;
        }
        else
        {
            outtake->adjustUp();
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

        // activate arm
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            arm->activate();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            arm->deactivate();
        }

        double pitch = drivebase->getIMU().get_pitch();

        // anti-tip loader deploy
        // only applies if robot is tipping forward
        if (pitch < PITCH_THRESHOLD)
        {
            loaderDeployed = true;
        }

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

        IntakeMode currentIntakeMode = intake->getIntakeMode();
        // pros::lcd::print(0, "Intake Mode: %s", (currentIntakeMode == IntakeMode::OFF) ? "OFF" : (currentIntakeMode == IntakeMode::FORWARD) ? "FORWARD"
        //                                                                                     : (currentIntakeMode == IntakeMode::REVERSE)   ? "REVERSE"
        //                                                                                     : (currentIntakeMode == IntakeMode::UNFOLD)    ? "UNFOLD"
        //
        pros::lcd::print(0, "Outtake Height: %s", (outtake->isHigh() ? "HIGH" : "LOW"));
        // pros::lcd::print(1, "Loader: %s", loaderDeployed ? "DEPLOYED" : "RETRACTED");
        // pros::lcd::print(2, "Tipping Forward: %s", (pitch < PITCH_THRESHOLD) ? "YES" : "NO");
        // pros::lcd::print(3, "Pitch: %f", pitch);

        pros::lcd::print(1, "Pose: (%f, %f, %f)", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().theta);
        pros::delay(50);
    }
}