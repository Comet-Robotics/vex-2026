#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "tasks/teleop.h"
#include "subsystems.h"

using namespace pros;

struct Test
{
    std::string name;
    std::function<void()> run;
    std::function<void()> init = []() {};
};

// variables for testing
// square test
int squareStep = 0;
const double squareSpeed = 30.0;
const double sideLength = 48.0;

// static friction test
double staticFrictionPower = 0.0;
double firstMovePower[4] = {0.0, 0.0, 0.0, 0.0};
double moved[4] = {false, false, false, false};

Controller controller(pros::E_CONTROLLER_MASTER);

enum class OpcontrolMode
{
    TEST,
    COMP
};

inline constexpr OpcontrolMode MODE = OpcontrolMode::TEST;

void opcontrol_initialize() {}

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

void testPodsDrive()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        drivebase->frontRight.setSpeeds(1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        drivebase->frontRight.setSpeeds(-1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        drivebase->frontLeft.setSpeeds(1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        drivebase->frontLeft.setSpeeds(-1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        drivebase->backLeft.setSpeeds(1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        drivebase->backLeft.setSpeeds(-1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        drivebase->backRight.setSpeeds(1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        drivebase->backRight.setSpeeds(-1, -1);
    }
    else
    {
        drivebase->frontRight.setSpeeds(0, 0);
        drivebase->frontLeft.setSpeeds(0, 0);
        drivebase->backRight.setSpeeds(0, 0);
        drivebase->backLeft.setSpeeds(0, 0);
    }
}

void testPodsRotation()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X))
    {
        drivebase->frontRight.setSpeeds(1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        drivebase->frontRight.setSpeeds(-1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        drivebase->frontLeft.setSpeeds(1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
    {
        drivebase->frontLeft.setSpeeds(-1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        drivebase->backLeft.setSpeeds(1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        drivebase->backLeft.setSpeeds(-1, 1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        drivebase->backRight.setSpeeds(1, -1);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        drivebase->backRight.setSpeeds(-1, 1);
    }
    else
    {
        drivebase->frontRight.setSpeeds(0, 0);
        drivebase->frontLeft.setSpeeds(0, 0);
        drivebase->backRight.setSpeeds(0, 0);
        drivebase->backLeft.setSpeeds(0, 0);
    }
}

void snapModules(double angle)
{
    for (int i = 0; i < 4; i++)
    {
        drivebase->getModule(i).setSpeedAndAngle(0, angle);
    }
    drivebase->update();
}

void motorHealthCheck()
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        for (int i = 0; i < 4; i++)
        {
            drivebase->getModule(i).setSpeeds(1.0, 1.0);
        }
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        for (int i = 0; i < 4; i++)
        {
            drivebase->getModule(i).setSpeeds(-1.0, -1.0);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            drivebase->getModule(i).setSpeeds(0, 0);
        }
    }
    for (int i = 0; i < 4; i++)
    {
        drivebase->getModule(i).calculateLinearSpeed();
    }
    pros::lcd::print(2, "Controls: Up = Full Forward, Down = Full Reverse");
    pros::lcd::print(3, "FR: %.2f  FL: %.2f", drivebase->frontRight.getLinearSpeed(), drivebase->frontLeft.getLinearSpeed());
    pros::lcd::print(4, "BL: %.2f  BR: %.2f", drivebase->backLeft.getLinearSpeed(), drivebase->backRight.getLinearSpeed());
    pros::lcd::print(5, "FL_MotorVel: %.2f %.2f", drivebase->frontLeft.topMotor.get_actual_velocity(), drivebase->frontLeft.bottomMotor.get_actual_velocity());
}

void squareTest()
{
    // drive in a 48 inch square to test odometry
    Pose2D current = drivebase->getPose();
    double fwd = 0, strafe = 0;

    switch (squareStep)
    {
    case 0:
        fwd = squareSpeed;
        if (current.x >= sideLength)
        {
            squareStep++;
        }
        break;
    case 1:
        strafe = squareSpeed;
        if (current.y <= -sideLength)
        {
            squareStep++;
        }
        break;
    case 2:
        fwd = -squareSpeed;
        if (current.x <= 0)
        {
            squareStep++;
        }
        break;
    case 3:
        strafe = squareSpeed;
        if (current.y >= 0)
        {
            squareStep = 0;
        }
        break;
    default:
        fwd = 0;
        strafe = 0;
        pros::lcd::print(1, "Invalid square step: %d", squareStep);
        break;
    }

    drivebase->setModuleSpeeds(fwd, strafe, 0, false);
    drivebase->update();
}

void staticFrictionTest()
{
    if (staticFrictionPower < 0.25)
    {
        staticFrictionPower += 0.0006;
    }

    for (int i = 0; i < 4; i++)
    {
        drivebase->getModule(i).setSpeeds(staticFrictionPower, staticFrictionPower);

        if (!moved[i] && std::abs(drivebase->getModule(i).getLinearSpeed()) > 1)
        {
            moved[i] = true;
            firstMovePower[i] = staticFrictionPower;
        }
    }

    pros::lcd::print(2, "CURRENT POWER: %.2f", staticFrictionPower);

    pros::lcd::print(3, "FR: %.2f  FL: %.2f", firstMovePower[0], firstMovePower[1]);
    pros::lcd::print(4, "BL: %.2f  BR: %.2f", firstMovePower[2], firstMovePower[3]);

    // determine if outlier exists
    double maxP, minP;
    for (int i = 0; i < 4; i++)
    {
        if (i == 0 || firstMovePower[i] > maxP)
        {
            maxP = firstMovePower[i];
        }
        if (i == 0 || firstMovePower[i] < minP)
        {
            minP = firstMovePower[i];
        }
    }
    if (maxP - minP > 0.1)
    {
        pros::lcd::print(5, "Outlier! Max: %.2f Min: %.2f", maxP, minP);
    }
    else
    {
        pros::lcd::print(5, "No outlier. Max: %.2f Min: %.2f", maxP, minP);
    }
}

void maxVelocityTest()
{
    if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        for (int i = 0; i < 4; i++)
        {
            drivebase->getModule(i).setSpeeds(0, 0);
        }
        pros::lcd::print(2, "Test starting on next UP press...");
        return;
    }
    int duration = 1000; // milliseconds
    int beginTime = pros::millis();

    // set all modules to full speed forward
    for (int i = 0; i < 4; i++)
    {
        drivebase->getModule(i).setSpeeds(1.0, 1.0);
    }
    double maxVels[4] = {0.0, 0.0, 0.0, 0.0};
    while (pros::millis() - beginTime < duration)
    {
        for (int i = 0; i < 4; i++)
        {
            double currentVel = std::abs(drivebase->getModule(i).getLinearSpeed());
            if (currentVel > maxVels[i])
            {
                maxVels[i] = currentVel;
            }
        }

        pros::lcd::print(2, "Max Vels - ");
        pros::lcd::print(3, "FR: %.2f  FL: %.2f", maxVels[0], maxVels[1]);
        pros::lcd::print(4, "BL: %.2f  BR: %.2f", maxVels[2], maxVels[3]);

        pros::delay(20);
    }

    for (int i = 0; i < 4; i++)
    {
        drivebase->getModule(i).setSpeeds(0, 0);
    }

    while (true)
    {
        pros::lcd::print(2, "Test complete. Max Vels - FR: %.2f, FL: %.2f, BL: %.2f, BR: %.2f", maxVels[0], maxVels[1], maxVels[2], maxVels[3]);
        pros::delay(1000);
    }
}

std::vector<Test> tests = {
    {
        "Normal Drive",
        drivebase_controls,
    },
    {
        "Test Pods Drive",
        testPodsDrive,
    },
    {
        "Test Pods Rotation",
        testPodsRotation,
    },
    {
        "Snap 0 deg",
        []()
        { snapModules(0); },
    },
    {
        "Snap 90 deg",
        []()
        { snapModules(90); },
    },
    {
        "Snap 180 deg",
        []()
        { snapModules(180); },
    },
    {
        "Snap 270 deg",
        []()
        { snapModules(270); },
    },
    {
        "Health Check (FULL POWER)",
        motorHealthCheck,
    },
    {
        "Square",
        squareTest,
        []()
        {
            squareStep = 0;
            drivebase->setPose({0, 0, 0});
            drivebase->tareIMU();
        },
    },
    {
        "Static Friction",
        staticFrictionTest,
        []()
        {
            staticFrictionPower = 0.0;
            for (int i = 0; i < 4; i++)
            {
                firstMovePower[i] = 0.0;
                moved[i] = false;
            }
        },
    },
    {
        "Max Velocity",
        maxVelocityTest,
    },
    {
        "Forward Slowly",
        []()
        {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
            {
                drivebase->setModuleSpeeds(constants::drivetrain::MAX_LINEAR_SPEED / 4.0, 0, 0, false);
            }
            else
            {
                drivebase->setModuleSpeeds(0, 0, 0, false);
            }

            drivebase->update();

            pros::lcd::print(2, "Odom: X: %1.2f Y: %1.2f H: %1.2f", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().heading);
        },
    }};

void opcontrolTest()
{
    int testIndex = 0;
    tests[testIndex].init();

    while (true)
    {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
        {
            testIndex = (testIndex + 1) % tests.size();
            tests[testIndex].init();
        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
        {
            testIndex = (testIndex - 1 + tests.size()) % tests.size();
            tests[testIndex].init();
        }

        pros::lcd::print(0, "[%d/%d] Test: %s", testIndex + 1, tests.size(), tests[testIndex].name.c_str());

        tests[testIndex].run();

        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void opcontrolComp()
{
    while (true)
    {
        drivebase_controls();
        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

void opcontrol()
{
    // drivebase.setAutonomous(false);

    switch (MODE)
    {
    case OpcontrolMode::TEST:
        return opcontrolTest();
    case OpcontrolMode::COMP:
        return opcontrolComp();
    default:
        break;
    }
}