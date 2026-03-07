#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::blocker;

class Blocker : public pros::adi::Pneumatics
{
public:
    Blocker() : pros::adi::Pneumatics(BLOCKER_PORT, false, false)
    {
        activate();
    }

    /**
     * Activates the blocker mechanism by setting the digital output to true. This will allow the robot to extend the blocker.
     */
    void activate()
    {
        set_value(HIGH);
    }

    /**
     * Deactivates the blocker mechanism by setting the digital output to false. This will retract the blocker.
     */
    void deactivate()
    {
        set_value(LOW);
    }

    /**
     * Toggles the state of the blocker mechanism. If the blocker is currently activated, it will be deactivated, and if it is currently deactivated, it will be activated.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
};