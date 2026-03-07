#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void autonomous_initialize()
{
}

void angularTest()
{
    double angle = 90;
    drivebase->setPose(0, 0, 0);
    std::vector<double> errors;
    std::vector<int> times;

    for (int i = 0; i < 12; i++)
    {
        int before = pros::millis();
        drivebase->turnToHeading(angle * (i + 1), 5000, {}, false);
        int after = pros::millis();
        times.push_back(after - before);
        errors.push_back(std::abs(normalizeAngleDeg(angle * (i + 1) - drivebase->getAngle())));
        pros::delay(100);
    }

    double totalError = 0;
    for (double error : errors)
    {
        totalError += error;
    }
    double averageError = totalError / errors.size();
    double stdDevError = 0;
    for (double error : errors)
    {
        stdDevError += (error - averageError) * (error - averageError);
    }
    stdDevError = sqrt(stdDevError / errors.size());

    int totalTime = 0;
    for (int time : times)
    {
        totalTime += time;
    }
    double averageTime = (double)totalTime / times.size();
    int stdDevTime = 0;
    for (int time : times)
    {
        stdDevTime += (time - averageTime) * (time - averageTime);
    }
    stdDevTime = sqrt(stdDevTime / times.size());
    pros::lcd::print(0, "Average Error: %f", averageError);
    pros::lcd::print(1, "Std Dev Error: %f", stdDevError);
    pros::lcd::print(2, "Average Time: %f ms", averageTime);
    pros::lcd::print(3, "Std Dev Time: %d ms", stdDevTime);
    pros::lcd::print(4, "PID: kP = %.2f, kI = %.2f, kD = %.2f", ANGULAR_CONTROLLER.kP, ANGULAR_CONTROLLER.kI, ANGULAR_CONTROLLER.kD);

    while (true)
    {
        pros::delay(10);
    }
}

void lateralTest()
{
    drivebase->setPose(0, 0, 0);
    double length = 24;
    std::vector<double> errors;
    std::vector<int> times;
    for (int i = 0; i < 12; i++)
    {
        int before = pros::millis();
        int target = 0;
        if (i % 2 != 0)
        {
            drivebase->moveToPoseComet(0, 0, 90, DEFAULT_TIMEOUT, {.forwards = false}, false); // move backwards
            target = 0;
        }
        else
        {
            drivebase->moveToPoseComet(0, length, 90, DEFAULT_TIMEOUT, {}, false); // move forwards
            target = length;
        }
        int after = pros::millis();
        errors.push_back(std::abs(target - drivebase->getPose().y));
        times.push_back(after - before);
        pros::delay(100);
    }

    double totalError = 0;
    for (double error : errors)
    {
        totalError += error;
    }
    double averageError = totalError / errors.size();
    double stdDevError = 0;
    for (double error : errors)
    {
        stdDevError += (error - averageError) * (error - averageError);
    }
    stdDevError = sqrt(stdDevError / errors.size());

    int totalTime = 0;
    for (int time : times)
    {
        totalTime += time;
    }
    double averageTime = (double)totalTime / times.size();
    int stdDevTime = 0;
    for (int time : times)
    {
        stdDevTime += (time - averageTime) * (time - averageTime);
    }
    stdDevTime = sqrt(stdDevTime / times.size());
    pros::lcd::print(0, "Average Error: %f", averageError);
    pros::lcd::print(1, "Std Dev Error: %f", stdDevError);
    pros::lcd::print(2, "Average Time: %f ms", averageTime);
    pros::lcd::print(3, "Std Dev Time: %d ms", stdDevTime);
    pros::lcd::print(4, "PID: kP = %.2f, kI = %.2f, kD = %.2f", LATERAL_CONTROLLER.kP, LATERAL_CONTROLLER.kI, LATERAL_CONTROLLER.kD);

    while (true)
    {
        pros::delay(10);
    }
}

void timeoutTest()
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        drivebase->moveToPoseComet(0, 24, 90, DEFAULT_TIMEOUT);
        while (drivebase->isInMotion())
        {
            pros::lcd::print(0, "Pose: (%f, %f, %f)", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().theta);
            pros::delay(20);
        }
        master.rumble("-");
        pros::delay(500);
        drivebase->turnToHeadingComet(270, DEFAULT_TIMEOUT);
        while (drivebase->isInMotion())
        {
            pros::lcd::print(0, "Pose: (%f, %f, %f)", drivebase->getPose().x, drivebase->getPose().y, drivebase->getPose().theta);
            pros::delay(20);
        }
        master.rumble("-");
        pros::delay(500);
    }
}

void autonomousSkillsSoloBevelGear()
{
    drivebase->setPoseComet(-46, 14, 90); // parallel with black part of park zone, just barely over it
    loader->activate();
    conveyor->adjustDown();
    blocker->activate();

    // drivebase->turnThenMoveToPoint(-46, 24, DEFAULT_TIMEOUT_LONG, {}, {}, false);
    drivebase->moveToPoseComet(-48, 48, 90, DEFAULT_TIMEOUT, {}, false);

    // obtain loader blocks
    drivebase->turnToPoint(-62, 48, DEFAULT_TIMEOUT, {}, false);
    conveyor->forwardSlow();
    drivebase->turnThenMoveToPoint(-62, 48, 1500, {}, {}, false);
    for (int i = 0; i < 12; i++)
    {
        drivebase->driveVoltage(300, 4000);
        pros::delay(350);
        drivebase->driveVoltage(150, 4000, false);
        pros::delay(200);
    }
    drivebase->driveVoltage(200, 4000, false);
    drivebase->setX(-58);
    conveyor->stop();
    conveyor->adjustUp();

    // score into long goal
    drivebase->turnThenMoveToPoint(-33, 49, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->forward();
    pros::delay(5000);
    conveyor->reverse();
    drivebase->driveVoltage(300, 4000);
    conveyor->forward();
    drivebase->driveVoltage(300, 4000, false);
    pros::delay(2000);
    conveyor->stop();
    drivebase->setX(-33);
    drivebase->setY(49);
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {}, {}, false);
    loader->deactivate();
    conveyor->adjustDown();

    // obtain blocks from side
    conveyor->forward();
    drivebase->turnThenMoveToPoint(-50, 63, 1300, {}, {}, false);
    pros::delay(500);
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->stop();
    conveyor->adjustUp();

    // score into long goal again
    drivebase->turnThenMoveToPoint(-33, 49, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->forward();
    pros::delay(4000);
    drivebase->setX(-33);
    drivebase->setY(49);

    // park
    drivebase->turnThenMoveToPoint(-42, 51, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    loader->activate();
    drivebase->turnThenMoveToPoint(-42, 0, DEFAULT_TIMEOUT_LONG, {}, {}, false);
    loader->deactivate();
    drivebase->turnToHeadingComet(180, DEFAULT_TIMEOUT, {}, false);
    drivebase->signedDrive(127, 0);
    while (drivebase->getIMU().get_pitch() > -2)
    {
        pros::delay(10);
    }
    pros::delay(500);
    drivebase->signedDrive(0, 0);
}

void autonomousSkills73BevelGear()
{
    // starting position
    drivebase->setPoseComet(-62.75, -16, -90); // against wall and just barely over loader
    loader->activate();
    conveyor->adjustDown();
    blocker->activate();

    // remove blocks from park zone
    // drivebase->turnThenMoveToPoint(-46, -24, DEFAULT_TIMEOUT_LONG, {}, {}, false);
    drivebase->moveToPoseComet(-48, -48, -90, DEFAULT_TIMEOUT_LONG, {}, false);
    drivebase->turnToPoint(-62, -48, DEFAULT_TIMEOUT, {}, false);
    conveyor->forward();
    drivebase->turnThenMoveToPoint(-62, -48, 1500, {}, {}, false);
    for (int i = 0; i < 12; i++)
    {
        drivebase->driveVoltage(300, 4000);
        pros::delay(350);
        drivebase->driveVoltage(150, 4000, false);
        pros::delay(200);
    }
    drivebase->driveVoltage(200, 4000, false);
    drivebase->setX(-58);
    conveyor->stop();
    conveyor->adjustUp();

    // score into long goal
    drivebase->turnThenMoveToPoint(-33, -49, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->forward();
    pros::delay(5000);
    conveyor->reverse();
    drivebase->driveVoltage(300, 4000);
    conveyor->forward();
    drivebase->driveVoltage(300, 4000, false);
    pros::delay(2000);
    conveyor->stop();
    drivebase->setX(-33);
    drivebase->setY(-49);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {}, {}, false);
    conveyor->adjustDown();
    loader->deactivate();

    // obtain blocks from side
    conveyor->forward();
    drivebase->turnThenMoveToPoint(-50, -63.5, 1300, {}, {}, false);
    pros::delay(500);
    drivebase->setY(-63.5);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->stop();
    conveyor->adjustUp();

    // score into long goal again
    drivebase->turnThenMoveToPoint(-33, -49, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->forward();
    pros::delay(4000);
    conveyor->stop();
    drivebase->setX(-33);
    drivebase->setY(-49);

    // move to other side of field
    drivebase->moveToPoseComet(-48, -60, 180, DEFAULT_TIMEOUT, {}, false);
    drivebase->turnThenMoveToPoint(60, -60, DEFAULT_TIMEOUT_LONG, {}, {}, false);
    drivebase->driveVoltage(1000, 6000, false);
    drivebase->setX(63);

    // obtain blocks from other loader
    drivebase->turnToPoint(48, -48, DEFAULT_TIMEOUT, {}, false);
    conveyor->forwardSlow();
    drivebase->turnThenMoveToPoint(62, -48, 1500, {}, {}, false);
    for (int i = 0; i < 12; i++)
    {
        drivebase->driveVoltage(300, 4000);
        pros::delay(350);
        drivebase->driveVoltage(150, 4000, false);
        pros::delay(200);
    }
    drivebase->driveVoltage(200, 4000, false);
    drivebase->setX(58);
    conveyor->stop();
    conveyor->adjustUp();

    // score into long goal from other side
    drivebase->turnThenMoveToPoint(33, 49, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    conveyor->forward();
    pros::delay(5000);
    conveyor->stop();
}

void autonomous()
{
    // angularTest();
    // lateralTest();
    // timeoutTest();
    // autonomousSkillsSoloBevelGear();
    autonomousSkills73BevelGear();
    // autonomous2v2Robot();
    // testGoalPush();
}