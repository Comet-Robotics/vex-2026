#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"

using namespace constants::park;

class Park : public pros::MotorGroup
{
public:
    Park() : pros::MotorGroup(std::vector<int8_t>(PARK_PORTS.begin(), PARK_PORTS.end()))
    {
        set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    }

    /**
     * Moves the park motors at max speed to park
     */
    inline void park() { this->move_voltage(MAX_PARK_SPEED); }

    /**
     * Moves the park motors to the setup position
     */
    inline void setup() { this->move_relative(-SETUP_POSITION, MAX_PARK_SPEED); }

    /**
     * Stops the park motors
     */
    inline void stop() { this->move_voltage(0); }

private:
};