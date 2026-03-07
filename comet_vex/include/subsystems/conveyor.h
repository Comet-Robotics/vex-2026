#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"

using namespace constants::conveyor;

class Conveyor : public pros::MotorGroup
{
public:
    Conveyor() : pros::MotorGroup(std::vector<int8_t>(CONVEYOR_PORTS.begin(), CONVEYOR_PORTS.end())),
                 heightAdjust(HEIGHT_ADJUST_PORT, false, false)
    {
        set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    }

    /**
     * Sets the conveyor motors to move forward at maximum speed.
     */
    inline void forward() { this->move_voltage(MAX_CONVEYOR_SPEED); }

    inline void forwardSlow() { this->move_voltage(SLOW_CONVEYOR_SPEED); }

    /**
     * Sets the conveyor motors to move in reverse at maximum speed.
     */
    inline void reverse() { this->move_voltage(-MAX_CONVEYOR_SPEED); }

    /**
     * Stops the conveyor motors.
     */
    inline void stop() { this->move_voltage(0); }

    /**
     * Toggles the forward movement of the conveyor motors. If the motors are currently moving forward, they will stop. If they are stopped or moving in reverse, they will start moving forward.
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
        heightAdjust.set_value(LOW);
        extended = true;
    }

    /**
     * Adjusts the height of the outtake mechanism downwards by deactivating the height adjust digital output.
     */
    void adjustDown()
    {
        heightAdjust.set_value(HIGH);
        extended = false;
    }

    /**
     * Gets the current state of the height adjust digital output, which can be used to determine the current height of the outtake mechanism.
     * @return true if the height adjust digital output is activated, indicating that the outtake mechanism is at its higher position, and false if it is deactivated, indicating that the outtake mechanism is at its lower position.
     */
    bool isHigh()
    {
        return extended;
    }

private:
    pros::adi::Pneumatics heightAdjust;
    bool extended = false;
};