#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"

using namespace constants::intake;

class Intake : public pros::MotorGroup
{
    public:
        Intake() : pros::MotorGroup(std::vector<int8_t>(INTAKE_PORTS.begin(), INTAKE_PORTS.end())){}

        inline void forward() {this->move_voltage(MAX_INTAKE_SPEED);}
        inline void reverse() {this->move_voltage(-MAX_INTAKE_SPEED);}
        inline void stop() {this->move_voltage(0);}

        void toggleForward() {
            if (this->get_voltage() > 0) {
                this->stop();
            } else {
                this->forward();
            }
        }

        void toggleReverse() {
            if (this->get_voltage() < 0) {
                this->stop();
            } else {
                this->reverse();
            }
        }
};

