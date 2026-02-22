#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::loader;

class Loader : public pros::adi::Pneumatics
{
public:
    Loader() : pros::adi::Pneumatics(LOADER_PORT, false, false)
    {
        deactivate();
    }

    /**
     * Activates the loader mechanism by setting the digital output to true. This will allow the robot to intake blocks from the loader.
     */
    void activate()
    {
        set_value(HIGH);
    }

    /**
     * Deactivates the loader mechanism by setting the digital output to false. This will retract the loader mechanism and stop it from intaking blocks.
     */
    void deactivate()
    {
        set_value(LOW);
    }

    /**
     * Toggles the state of the loader mechanism. If the loader is currently activated, it will be deactivated, and if it is currently deactivated, it will be activated.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
    bool deployed = false;
};