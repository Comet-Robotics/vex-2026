#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "tasks/teleop.h"
#include "tasks/tests.h"
#include "subsystems.h"

using namespace pros;

bool isTestMode = false;
bool loaderDeployed = false;

void opcontrol_initialize() {}

void controls()
{
    drivebase_controls();

    // intake/outtake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) // intaking from loader
    {
        loaderDeployed = true;
        conveyor->forward();
        blocker->block();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // intaking from floor
    {
        loaderDeployed = false;
        conveyor->forward();
        blocker->block();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) // scoring
    {
        loaderDeployed = false;
        conveyor->forward();
        blocker->unblock();
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) // reverse slow
    {
        loaderDeployed = false;
        conveyor->reverseSlow();
    }
    else // stop
    {
        loaderDeployed = false;
        conveyor->stop();
    }

    // outtake height adjust
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        conveyor->adjustUp();
    }
    else
    {
        conveyor->adjustDown();
    }

    // // deploy loader
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    // {
    //     loaderDeployed = true;
    // }
    // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    // {
    //     loaderDeployed = false;
    // }

    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    // {
    //     loaderDeployed = !loaderDeployed;
    // }

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
    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
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
}

void drivebase_controls()
{
    double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * constants::drivetrain::MAX_LINEAR_SPEED;
    double strafe = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0 * constants::drivetrain::MAX_LINEAR_SPEED;
    double rotation = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * constants::drivetrain::MAX_ANGULAR_SPEED;

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        drivebase->tareIMU();
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        drivebase->resetPose();
    }

    // pros::lcd::print(1, "Fwd: %1.2f Str: %1.2f Rot: %1.2f", forward, strafe, rotation);
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) // X wheels
    {
        drivebase->xWheels();
    }
    else
    {
        drivebase->setModuleSpeeds(forward, strafe, rotation, constants::drivetrain::FIELD_CENTRIC_DEFAULT);
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    {
        drivebase->toggleHeadingHold();
    }

    drivebase->update();

    pros::lcd::print(0, "Temp");
    pros::lcd::print(1, "1: %1.2f 2: %1.2f 3: %1.2f 4: %1.2f", drivebase->frontRight.topMotor.get_temperature(), drivebase->frontRight.bottomMotor.get_temperature(),
                     drivebase->backRight.bottomMotor.get_temperature(), drivebase->backRight.topMotor.get_temperature());
    pros::lcd::print(2, "5: %1.2f 6: %1.2f 7: %1.2f 8: %1.2f", drivebase->frontLeft.bottomMotor.get_temperature(), drivebase->frontLeft.topMotor.get_temperature(),
                     drivebase->backLeft.topMotor.get_temperature(), drivebase->backLeft.bottomMotor.get_temperature());
    pros::lcd::print(3, "Odom: X: %1.2f Y: %1.2f H: %1.2f", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().heading);
    // pros::lcd::print(3, "Current");
    // pros::lcd::print(4, "1: %1.2f 2: %1.2f 3: %1.2f 4: %1.2f", drivebase->frontRight.topMotor.get_current_draw(), drivebase->frontRight.bottomMotor.get_current_draw(),
    //                  drivebase->backRight.bottomMotor.get_current_draw(), drivebase->backRight.topMotor.get_current_draw());
    // pros::lcd::print(5, "5: %1.2f 6: %1.2f 7: %1.2f 8: %1.2f", drivebase->frontLeft.bottomMotor.get_current_draw(), drivebase->frontLeft.topMotor.get_current_draw(),
    //                  drivebase->backLeft.topMotor.get_current_draw(), drivebase->backLeft.bottomMotor.get_current_draw());

    if (drivebase->isOverTemperature())
    {
        controller.rumble("-");
    }
}

void opcontrol()
{
    tests_initialize();

    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            isTestMode = !isTestMode;
            if (isTestMode)
            {
                pros::lcd::print(0, "Switched to TEST MODE");
            }
            else
            {
                pros::lcd::print(0, "Switched to COMP MODE");
                // clear screen when returning
                for (int i = 1; i < 7; i++)
                {
                    pros::lcd::clear_line(i);
                }
            }
        }

        if (isTestMode)
        {
            runTestModeIteration(controller);
        }
        else
        {
            controls();
        }

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}