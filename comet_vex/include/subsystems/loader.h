#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::loader;

class Loader : public pros::adi::DigitalOut
{
public:
    Loader() : pros::adi::DigitalOut(LOADER_PORT)
    {
        deactivate();
    }

    /**
     * Activates the loader mechanism by setting the digital output to true. This will allow the robot to intake blocks from the loader.
     */
    void activate()
    {
        set_value(true);
        deployed = true;
    }

    /**
     * Deactivates the loader mechanism by setting the digital output to false. This will retract the loader mechanism and stop it from intaking blocks.
     */
    void deactivate()
    {
        set_value(false);
        deployed = false;
    }

    /**
     * Toggles the state of the loader mechanism. If the loader is currently activated, it will be deactivated, and if it is currently deactivated, it will be activated.
     */
    void toggle()
    {
        deployed = !deployed;
        set_value(deployed);
    }

private:
    bool deployed = false;
};