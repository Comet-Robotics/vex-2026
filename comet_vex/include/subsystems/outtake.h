#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"
#include <cstdint>

using namespace constants::outtake;

class Outtake : public pros::MotorGroup
{
public:
    Outtake() : pros::MotorGroup(std::vector<int8_t>(OUTTAKE_PORTS.begin(), OUTTAKE_PORTS.end())),
                heightAdjust(HEIGHT_ADJUST_PORT, false, false) {}

    /**
     * Sets the outtake motors to move forward at maximum speed.
     */
    inline void forward() { this->move_voltage(MAX_OUTTAKE_SPEED); }

    /**
     * Sets the outtake motors to move in reverse at maximum speed.
     */
    inline void reverse() { this->move_voltage(-MAX_OUTTAKE_SPEED); }

    /**
     * Stops the outtake motors.
     */
    inline void stop() { this->move_voltage(0); }

    /**
     * Toggles the forward movement of the outtake motors. If the motors are currently moving forward, they will stop. If they are stopped or moving in reverse, they will start moving forward.
     */
    void toggleForward()
    {
        if (this->get_voltage() > 0)
        {
            this->stop();
        }
        else
        {
            this->forward();
        }
    }

    /**
     * Toggles the reverse movement of the outtake motors. If the motors are currently moving in reverse, they will stop. If they are stopped or moving forward, they will start moving in reverse.
     */
    void toggleReverse()
    {
        if (this->get_voltage() < 0)
        {
            this->stop();
        }
        else
        {
            this->reverse();
        }
    }

    /**
     * Adjusts the height of the outtake mechanism upwards by activating the height adjust digital output.
     */
    void adjustUp()
    {
        heightAdjust.set_value(HIGH);
    }

    /**
     * Adjusts the height of the outtake mechanism downwards by deactivating the height adjust digital output.
     */
    void adjustDown()
    {
        heightAdjust.set_value(LOW);
    }

private:
    pros::adi::Pneumatics heightAdjust;
};