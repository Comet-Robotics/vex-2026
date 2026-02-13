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
}

void autonomous() {
    angularTest();
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

