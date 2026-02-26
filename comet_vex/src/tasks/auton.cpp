#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void autonomous_initialize()
{
    drivebase->calibrateChassis(true);
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

void autonomousSkills73Nobot()
{
    drivebase->setPoseComet(-46, 0, 90);
    intake->setIntakeMode(IntakeMode::UNFOLD);

    // remove blocks from park zone
    // TODO: deploy arm then wait a bit
    drivebase->turnThenMoveToPoint(-46, 24, DEFAULT_TIMEOUT, {}, {}, false);
    // TODO: raise arm
    drivebase->moveToPoseComet(-48, 48, 90, DEFAULT_TIMEOUT, {}, false);

    // obtain loader blocks
    loader->activate();
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-62, 48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    loader->deactivate();
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    // score into long goal
    drivebase->turnThenMoveToPoint(-31, 48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(5000);
    outtake->stop();
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);

    // obtain blocks from side
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-48, 65, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(500);
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    // score into long goal again
    drivebase->turnThenMoveToPoint(-31, 48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(2000);
    outtake->stop();

    // park
    drivebase->moveToPoseComet(-62, 24, 90, DEFAULT_TIMEOUT, {.forwards = false}, false);
    drivebase->turnThenMoveToPoint(-62, 6, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
}

void autonomousSkills73Robot()
{
    // starting postion
    drivebase->setPoseComet(-50, -15, -90);
    intake->setIntakeMode(IntakeMode::UNFOLD);

    // get blocks from loader
    drivebase->moveToPoseComet(-50, -48, -90, DEFAULT_TIMEOUT, {}, false);
    drivebase->turnToPoint(-63, -48, DEFAULT_TIMEOUT, {}, false);
    loader->activate();
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->driveFullVoltage(1000);
    pros::delay(5000);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    loader->deactivate();
    intake->setIntakeMode(IntakeMode::OFF);

    // score in long goal
    outtake->adjustUp();
    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(5000);
    outtake->stop();
    outtake->adjustDown();
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);

    // get blocks from side
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-48, -65, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(500);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    // score in long goal again
    outtake->adjustUp();
    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(2000);
    outtake->adjustDown();
    outtake->stop();

    // park
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    drivebase->turnToHeadingComet(90, DEFAULT_TIMEOUT);
    drivebase->moveToPoseComet(-62, -24, 90, DEFAULT_TIMEOUT, {}, false);
    drivebase->turnThenMoveToPoint(-62, -6, DEFAULT_TIMEOUT, {}, {}, false);
}

void autonomous2v2Nobot()
{
    drivebase->setPoseComet(-55, 15, 90);
    intake->setIntakeMode(IntakeMode::UNFOLD);

    drivebase->moveToPoseComet(-48, 48, 90, DEFAULT_TIMEOUT, {}, false);
    loader->activate();
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-63, 48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    loader->deactivate();
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    drivebase->turnThenMoveToPoint(-31, 48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(5000);
    intake->setIntakeMode(IntakeMode::OFF);
    outtake->stop();
}

void autonomous2v2Robot()
{
    drivebase->setPoseComet(-55, -15, -90);
    intake->setIntakeMode(IntakeMode::UNFOLD);

    drivebase->moveToPoseComet(-48, -48, -90, DEFAULT_TIMEOUT, {}, false);
    loader->activate();
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-63, -48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    loader->deactivate();
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(5000);
    intake->setIntakeMode(IntakeMode::OFF);
    outtake->stop();
}

void autonomous()
{
    // angularTest();
    // lateralTest();
    // timeoutTest();
    autonomousSkills73Robot();
    // autonomousSkills73Nobot();
    // autonomous2v2Robot();
}