#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"
#include "pros/llemu.hpp"

using namespace constants::intake;

enum class IntakeMode
{
    OFF,
    FORWARD,
    REVERSE,
    UNFOLD
};

class Intake : public pros::MotorGroup
{
public:
    Intake() : pros::MotorGroup(std::vector<int8_t>(INTAKE_PORTS.begin(), INTAKE_PORTS.end())) {}

    /**
     * Sets the intake motors to move forward at maximum speed, allowing the robot to intake blocks.
     */
    inline void forward()
    {
        this->move_voltage(MAX_INTAKE_SPEED);
    }

    /**
     * Sets the intake motors to move in reverse at maximum speed, allowing the robot to outtake blocks or unfold the intake mechanism.
     */
    inline void reverse()
    {
        this->move_voltage(-MAX_INTAKE_SPEED);
    }

    /**
     * Stops the intake motors.
     */
    inline void stop()
    {
        this->move_voltage(0);
    }

    /**
     * Toggles the forward movement of the intake motors. If the motors are currently moving forward, they will stop. If they are stopped or moving in reverse, they will start moving forward.
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
     * Toggles the reverse movement of the intake motors. If the motors are currently moving in reverse, they will stop. If they are stopped or moving forward, they will start moving in reverse.
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
     * Sets the intake mode of the intake mechanism for the intake task.
     * @param mode The desired intake mode to set for the intake mechanism. This will determine the behavior of the intake motors in the intake task.
     */
    void setIntakeMode(IntakeMode mode)
    {
        this->intakeMode.store(mode);
        firstRun = true;
    }

    double getIntakeMotorTemp()
    {
        return this->get_temperature();
    }

    /**
     * Gets the current intake mode of the intake mechanism.
     * @return The current intake mode.
     */
    IntakeMode getIntakeMode()
    {
        return this->intakeMode.load();
    }

    /**
     * The intake task that continuously runs in the background to control the intake motors based on the current intake mode. This task checks the intake mode and sets the motor behavior accordingly, including handling jam detection and de-jamming if necessary.
     */
    void intakeTask()
    {
        while (true)
        {
            switch (this->intakeMode.load())
            {
            case IntakeMode::OFF:
                this->stop();
                break;
            case IntakeMode::FORWARD:
                // if (isJammed())
                // {
                //     runDeJam();
                // }
                // else
                // {
                //     this->forward();
                // }

                // this->forward();
                break;
            case IntakeMode::REVERSE:
                this->reverse();
                break;
            case IntakeMode::UNFOLD:
                if (firstRun)
                {
                    this->reverse();
                    pros::delay(200);
                    firstRun = false;
                }
                this->stop();
                break;
            }
            pros::delay(50);
        }
    }

    IntakeMode getIntakeMode()
    {
        return this->intakeMode.load();
    }

    /**
     * Checks if the intake mechanism is currently jammed by comparing the current draw and voltage against predefined thresholds. If the current draw exceeds the JAM_CURRENT_THRESHOLD and the voltage exceeds the JAM_VOLTAGE_THRESHOLD, it is likely that the intake is jammed and requires de-jamming.
     * @return true if the intake is likely jammed, false otherwise
     */
    bool isJammed()
    {
        // pros::delay(10);
        // pros::lcd::print(2, "Current Draw: %d mA", this->get_current_draw());
        // pros::delay(10);
        // // pros::lcd::print(3, "Voltage: %d mV", this->get_voltage());
        // pros::lcd::print(3, "Jammed: %s", (this->get_current_draw() > JAM_CURRENT_THRESHOLD && this->get_voltage() > JAM_VOLTAGE_THRESHOLD) ? "YES" : "NO");
        return this->get_current_draw() > JAM_CURRENT_THRESHOLD;
    }

    /**
     * Runs a de-jamming routine by briefly reversing the intake motors and then returning to forward movement. This can help to clear any jams in the intake mechanism and allow it to continue functioning properly.
     */
    void runDeJam()
    {
        this->reverse();
        pros::delay(150);
        this->forward();
        pros::delay(150);
    }

private:
    std::atomic<IntakeMode> intakeMode{IntakeMode::OFF};
    bool firstRun = true;
};
