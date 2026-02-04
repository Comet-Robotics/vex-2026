#include "tasks/auton.h"
#include "subsystems.h"
#include "subsystems/drivebase.h"
#include "pros/llemu.hpp"

void autonomous_initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "before calibrate imu");
    drivebase->calibrateIMU();
    pros::lcd::print(0, "after calibrate imu");

    drivebase->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void autonomous() {
    pros::lcd::print(0, "before gotopose");
    drivebase->goToPoseUnicycle(Pose2D(12, 24, 0));
    pros::lcd::print(0, "after gotopose");
}