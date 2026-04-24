// #include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "tasks/teleop.h"
#include "tasks/tests.h"
#include "subsystems.h"
#include "constants.h"
#include <cstdio>

using namespace pros;

bool isTestMode = false;
bool loaderDeployed = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

int i = 0;

void controls()
{
    drivebase_controls();

    if (controller.get_digital(constants::controls::INTAKE_LOADER)) // intaking from loader
    {
        loaderDeployed = true;
        conveyor->intake();
        blocker->block();
    }
    else if (controller.get_digital(constants::controls::INTAKE_FLOOR)) // intaking from floor
    {
        loaderDeployed = false;
        conveyor->intake();
        blocker->block();
    }
    else if (controller.get_digital(constants::controls::SCORE)) // scoring
    {
        loaderDeployed = false;
        conveyor->score();
        blocker->unblock();
        drivebase->xWheels();
    }
    else if (controller.get_digital(constants::controls::OUTTAKE_LOW)) // reverse
    {
        loaderDeployed = false;
        conveyor->reverse();
    }
    else // stop
    {
        loaderDeployed = false;
        conveyor->stop();
    }

    // outtake height adjust
    if (controller.get_digital(constants::controls::ADJUST_DOWN))
    {
        conveyor->adjustDown();
    }
    else
    {
        conveyor->adjustUp();
    }

    if (loaderDeployed)
    {
        loader->activate();
    }
    else
    {
        loader->deactivate();
    }

    if (controller.get_digital(constants::controls::PARK))
    {
        park->park();
    }
    else if (controller.get_digital(constants::controls::UNPARK))
    {
        park->unpark();
    }
    else
    {
        park->stop();
    }

    if (controller.get_digital(constants::controls::WINGS_DOWN))
    {
        wings->down();
    }
    else if (controller.get_digital(constants::controls::WINGS_UP))
    {
        wings->up();
    }

    if (i % 10 == 0)
    {
        printf("y: %1.2f, dist: %1.2f\n", drivebase->getDistanceOffset(), drivebase->getDistance());
    }
    i++;

    // park->print_temperatures();
}

void opcontrol_initialize() {}

void drivebase_controls()
{
    double forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * constants::drivetrain::MAX_LINEAR_SPEED;
    double strafe = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0 * constants::drivetrain::MAX_LINEAR_SPEED;
    double rotation = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * constants::drivetrain::MAX_ANGULAR_SPEED;

    if (controller.get_digital(constants::controls::RESET_POSE))
    {
        drivebase->resetPose();
    }

    // // pros::lcd::print(x, "Fwd: %1.2f Str: %1.2f Rot: %1.2f", forward, strafe, rotation);
    if (controller.get_digital(constants::controls::X_WHEELS)) // X wheels
    {
        drivebase->xWheels();
    }
    else
    {
        drivebase->setModuleSpeeds(forward, strafe, rotation, constants::drivetrain::FIELD_CENTRIC_DEFAULT);
    }

    if (controller.get_digital_new_press(constants::controls::HEADING_HOLD_TOGGLE))
    {
        drivebase->toggleHeadingHold();
    }

    drivebase->update();

    // // pros::lcd::print(x, "Temp");
    if (i % 10 == 0)
    {
        printf("1: %1.2f 2: %1.2f 3: %1.2f 4: %1.2f ", drivebase->frontRight.topMotor.get_temperature(), drivebase->frontRight.bottomMotor.get_temperature(),
                         drivebase->backRight.bottomMotor.get_temperature(), drivebase->backRight.topMotor.get_temperature());
        printf("5: %1.2f 6: %1.2f 7: %1.2f 8: %1.2f\n", drivebase->frontLeft.bottomMotor.get_temperature(), drivebase->frontLeft.topMotor.get_temperature(),
                         drivebase->backLeft.topMotor.get_temperature(), drivebase->backLeft.bottomMotor.get_temperature());
    }
    // // pros::lcd::print(x, "Odom: X: %1.2f Y: %1.2f H: %1.2f", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().heading);

    // // pros::lcd::print(x, "Current");
    // // pros::lcd::print(x, "1: %1.2f 2: %1.2f 3: %1.2f 4: %1.2f", drivebase->frontRight.topMotor.get_current_draw(), drivebase->frontRight.bottomMotor.get_current_draw(),
    //                  drivebase->backRight.bottomMotor.get_current_draw(), drivebase->backRight.topMotor.get_current_draw());
    // // pros::lcd::print(x, "5: %1.2f 6: %1.2f 7: %1.2f 8: %1.2f", drivebase->frontLeft.bottomMotor.get_current_draw(), drivebase->frontLeft.topMotor.get_current_draw(),
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
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
        // {
        //     isTestMode = !isTestMode;
        //     if (isTestMode)
        //     {
        //         // pros::lcd::print(x, "Switched to TEST MODE");
        //     }
        //     else
        //     {
        //         // pros::lcd::print(x, "Switched to COMP MODE");
        //     }
        // }

        // if (isTestMode)
        // {
        //     runTestModeIteration(controller);
        // }
        // else
        // {
        //     controls();
        // }

        controls();

        // drivebase_controls();

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}