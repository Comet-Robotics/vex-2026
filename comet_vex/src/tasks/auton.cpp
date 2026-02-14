#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"



void autonomous_initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "before calibrate imu");
    drivebase->calibrateChassis(true);
    pros::lcd::print(0, "after calibrate imu");

    drivebase->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    new pros::Task([=]() { intake->intakeTask(); });

}

void autonomous() {
    // angularTest();
    // lateralTest();
}

void angularTest() {
    drivebase->setPose(0, 0, 0);
    drivebase->turnToHeading(90, 100000);
}

void lateralTest() {
    drivebase->setPose(0, 0, 0);
    drivebase->moveToPoint(0, 24, 100000);
}

void autonomousSkills73NotARobot() {
    drivebase->setPoseComet(-46, 0, 90);

    // remove blocks from park zone
    // TODO: deploy arm then wait a bit
    drivebase->turnThenMoveToPoint(-46, 24, DEFAULT_TIMEOUT, {}, {}, false);
    // TODO: raise arm
    drivebase->moveToPoseComet(-48, 48, 90, DEFAULT_TIMEOUT, {}, false);
    
    // obtain loader blocks
    // TODO: deploy loader intake
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-62, 48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    drivebase->turnThenMoveToPoint(-48, 48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF);
    
    // score into long goal
    drivebase->turnThenMoveToPoint(-31, 48, DEFAULT_TIMEOUT, {}, {}, false);
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
    outtake->forward();
    pros::delay(2000);
    outtake->stop();

    // park
    drivebase->moveToPoseComet(-62, 24, 90, DEFAULT_TIMEOUT, {.forwards = false}, false);
    drivebase->turnThenMoveToPoint(-62, 6, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
}

void autonomousSkills73Robot() {
    // starting postion
    drivebase->setPoseComet(-55, -15, 90);

    // get blocks from loader
    drivebase->moveToPoseComet(-48, -48, -90, DEFAULT_TIMEOUT, {}, false);
    // TODO: deploy loader intake
    intake->setIntakeMode(IntakeMode::FORWARD);
    drivebase->turnThenMoveToPoint(-63, -48, DEFAULT_TIMEOUT, {}, {}, false);
    pros::delay(1000);
    drivebase->turnThenMoveToPoint(-48, -48, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
    intake->setIntakeMode(IntakeMode::OFF); 
    
    // score in long goal
    drivebase->turnThenMoveToPoint(-31, -48, DEFAULT_TIMEOUT, {}, {}, false);
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
    outtake->forward(); 
    pros::delay(2000);
    outtake->stop();
    
    // park
    drivebase->moveToPoseComet(-62, -24, 90, DEFAULT_TIMEOUT, {.forwards = false}, false);
    drivebase->turnThenMoveToPoint(-62, -6, DEFAULT_TIMEOUT, {.forwards = false}, {.forwards = false}, false);
}