#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void autonomous_initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "before calibrate imu");
    drivebase->calibrateIMU();
    pros::lcd::print(0, "after calibrate imu");
}

void autonomous() {
    pros::lcd::print(0, "before gotopose");
    drivebase->goToPose(Pose2D(0, 24, 0));
    pros::lcd::print(0, "after gotopose");
}