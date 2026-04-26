#pragma once

#include "constants.h"
#include "pros/motors.hpp"

using namespace constants::park;

class Park
{
public:
    Park() : m1(PARK_PORTS[0]),
             m2(PARK_PORTS[1])
    {
        m1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        m2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    /**
     * Moves the park motors at max speed to park
     */
    inline void park()
    {
        m1.move_voltage(MAX_PARK_SPEED);
        m2.move_voltage(MAX_PARK_SPEED);
    }

    inline void unpark()
    {
        m1.move_voltage(-MAX_PARK_SPEED);
        m2.move_voltage(-MAX_PARK_SPEED);
    }

    /**
     * Moves the park motors to the setup position
     */
    inline void setup()
    {
        m1.move_relative(SETUP_POSITION, MAX_PARK_SPEED);
        m2.move_relative(SETUP_POSITION, MAX_PARK_SPEED);
    }

    /**
     * Stops the park motors
     */
    inline void stop()
    {
        m1.move_voltage(0);
        m2.move_voltage(0);
    }

    inline void print_temperatures()
    {
        int temp1 = m1.get_temperature();
        int temp2 = m2.get_temperature();
        printf("Park Motor Temperatures: %d, %d\n", temp1, temp2);
    }

private:
    pros::Motor m1;
    pros::Motor m2;
};