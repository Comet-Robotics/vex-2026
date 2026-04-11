#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "constants.h"

using namespace constants::loader;

class Loader : public pros::adi::Pneumatics
{
public:
    Loader() : pros::adi::Pneumatics(LOADER_PORT, true, false),
               loaderMotors(std::vector<int8_t>(LOADER_MOTOR_PORTS.begin(), LOADER_MOTOR_PORTS.end()))
    {
        loaderMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    }

    void forward() { loaderMotors.move_voltage(12000); }

    void reverse() { loaderMotors.move_voltage(-12000); }

    void stop() { loaderMotors.move_voltage(0); }

    /**
     * Activates the loader mechanism by setting the digital output to true. This will allow the robot to intake blocks from the loader.
     */
    void activate()
    {
        set_value(LOW);
    }

    /**
     * Deactivates the loader mechanism by setting the digital output to false. This will retract the loader mechanism and stop it from intaking blocks.
     */
    void deactivate()
    {
        set_value(HIGH);
    }

    /**
     * Toggles the state of the loader mechanism. If the loader is currently activated, it will be deactivated, and if it is currently deactivated, it will be activated.
     */
    void toggle()
    {
        set_value(!is_extended());
    }

private:
    pros::MotorGroup loaderMotors;
};