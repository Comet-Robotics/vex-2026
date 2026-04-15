#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::wings;

class Wings : public pros::adi::Pneumatics
{
public:
    Wings() : pros::adi::Pneumatics(WINGS_PORT, false, false)
    {
        up();
    }

    /**
     * Lowers the wings
     */
    void down()
    {
        set_value(HIGH);
    }

    /**
     * Raises the wings
     */
    void up()
    {
        set_value(LOW);
    }

    /**
     * Toggles the state of the wings mechanism. If the wings are currently down, they will be raised, and if they are currently up, they will be lowered.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
};