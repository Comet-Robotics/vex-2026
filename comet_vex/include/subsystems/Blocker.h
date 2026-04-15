#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::blocker;

class Blocker : public pros::adi::Pneumatics
{
public:
    Blocker() : pros::adi::Pneumatics(BLOCKER_PORT, false, false)
    {
        block();
    }

    /**
     * Blocks the end of the conveyor
     */
    void block()
    {
        set_value(HIGH);
    }

    /**
     * Unblocks the end of the conveyor
     */
    void unblock()
    {
        set_value(LOW);
    }

    /**
     * Toggles the state of the blocker mechanism. If the blocker is currently blocked, it will be unblocked, and if it is currently unblocked, it will be blocked.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
};