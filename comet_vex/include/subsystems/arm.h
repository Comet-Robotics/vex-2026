#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::arm;

class Arm : public pros::adi::Pneumatics
{
public:
    Arm() : pros::adi::Pneumatics(ARM_PORT, false, true)
    {
        deactivate();
    }

    /**
     * Activates the arm mechanism by setting the digital output to true. This will allow the robot to extend the arm.
     */
    void activate()
    {
        set_value(HIGH);
    }

    /**
     * Deactivates the arm mechanism by setting the digital output to false. This will retract the arm.
     */
    void deactivate()
    {
        set_value(LOW);
    }

    /**
     * Toggles the state of the arm mechanism. If the arm is currently activated, it will be deactivated, and if it is currently deactivated, it will be activated.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
};