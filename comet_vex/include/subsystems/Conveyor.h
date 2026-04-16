#pragma once

#include "constants.h"
#include "pros/motors.hpp" // Use individual motors instead of motor_group
#include "pros/adi.hpp"

using namespace constants::conveyor;

class Conveyor
{
public:
    // Instantiate individual motors directly from your constants array
    Conveyor() : intakeMotor(CONVEYOR_PORTS[0]),
                 conveyorMotor(CONVEYOR_PORTS[1]),
                 outtakeMotor(CONVEYOR_PORTS[2]),
                 heightAdjust(HEIGHT_ADJUST_PORT, false, false)
    {
        intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        conveyorMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        outtakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }

    inline void move_voltage(int voltage)
    {
        intakeMotor.move_voltage(voltage);
        conveyorMotor.move_voltage(voltage);
        outtakeMotor.move_voltage(voltage);
    }

    inline int get_voltage()
    {
        return intakeMotor.get_voltage();
    }

    inline void intake()
    {
        intakeMotor.move_voltage(MAX_CONVEYOR_SPEED);
        conveyorMotor.move_voltage(MAX_CONVEYOR_SPEED);
        outtakeMotor.move_voltage(SLOW_CONVEYOR_SPEED); // Slower speed for outtake motor during intake
    }

    inline void score()
    {
        this->move_voltage(MAX_CONVEYOR_SPEED);
    }

    inline void forwardSlow() { this->move_voltage(SLOW_CONVEYOR_SPEED); }
    inline void reverse() { this->move_voltage(-MAX_CONVEYOR_SPEED); }
    inline void reverseSlow() { this->move_voltage(-SLOW_REVERSE_SPEED); }
    inline void stop() { this->move_voltage(0); }

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

    void adjustUp()
    {
        heightAdjust.set_value(LOW);
        extended = true;
    }

    void adjustDown()
    {
        heightAdjust.set_value(HIGH);
        extended = false;
    }

    bool isHigh()
    {
        return extended;
    }

private:
    pros::Motor intakeMotor;
    pros::Motor conveyorMotor;
    pros::Motor outtakeMotor;
    pros::adi::Pneumatics heightAdjust;
    bool extended = false;
};