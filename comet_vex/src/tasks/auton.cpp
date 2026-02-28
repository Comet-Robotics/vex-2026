#include "tasks/auton.h"
#include "pros/llemu.hpp"
#include "pros/motor_group.hpp"

void autonomous_initialize()
{
}

void autonomous()
{
    pros::MotorGroup left({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
    pros::MotorGroup right({-11, -12, -13, -14, -15, -16, -17, -18, -19, -20});

    left.move_voltage(4000);
    right.move_voltage(4000);
    pros::delay(625);
    left.move_voltage(0);
    right.move_voltage(0);
}