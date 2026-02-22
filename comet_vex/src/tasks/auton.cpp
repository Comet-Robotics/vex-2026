#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void autonomous_initialize()
{
    pros::lcd::print(1, "Initializing autonomous...");

    new pros::Task([=]()
                   { intake->intakeTask(); });
}

void angularTest()
{
    double angle = 90;
    drivebase->setPose(0, 0, 0);
    std::vector<double> errors;

    for (int i = 0; i < 10; i++)
    {
        drivebase->turnToHeading(angle * (i + 1), 5000, {}, false);
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
    pros::lcd::print(0, "Average Error: %f", averageError);
    pros::lcd::print(1, "Std Dev Error: %f", stdDevError);

    while (true)
    {
        pros::delay(10);
    }
}

void lateralTest()
{
    drivebase->setPose(0, 0, 0);
    drivebase->moveToPoint(0, 24, 100000);
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
    drivebase->setPoseComet(-55, -15, -90);
    intake->setIntakeMode(IntakeMode::UNFOLD);

    // get blocks from loader
    drivebase->moveToPoseComet(-48, -48, -90, DEFAULT_TIMEOUT, {}, false);
    loader->activate();
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-63, -48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    loader->deactivate();
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    // score in long goal
    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(5000);
    outtake->stop();
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);

    // get blocks from side
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-48, -65, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(500);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);

    // score in long goal again
    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
    intake->setIntakeMode(IntakeMode::FORWARD);
    outtake->forward();
    pros::delay(2000);
    outtake->stop();

    // park
    drivebase->moveToPoseComet(-62, -24, 90, DEFAULT_TIMEOUT, {.forwards = false}, false);
    drivebase->turnThenMoveToPoint(-62, -6, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
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
    autonomousSkills73Robot();
}