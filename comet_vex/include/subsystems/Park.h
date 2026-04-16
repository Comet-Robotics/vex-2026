#pragma once

#include "constants.h"
#include "pros/motors.hpp" // Swapped from motor_group.hpp

using namespace constants::park;

class Park
{
public:
    // Initialize the two motors directly from your port array
    Park() : m1(PARK_PORTS[0]),
             m2(PARK_PORTS[1])
    {
        m1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        m2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    /**
     * Moves the park motors at max speed to park
     */
    inline void park()
    {
        m1.move_voltage(MAX_PARK_SPEED);
        m2.move_voltage(MAX_PARK_SPEED);
    }

    /**
     * Moves the park motors to the setup position
     */
    inline void setup()
    {
        m1.move_relative(-SETUP_POSITION, MAX_PARK_SPEED);
        m2.move_relative(-SETUP_POSITION, MAX_PARK_SPEED);
    }

    /**
     * Stops the park motors
     */
    inline void stop()
    {
        m1.move_voltage(0);
        m2.move_voltage(0);
    }

private:
    pros::Motor m1;
    pros::Motor m2;
};