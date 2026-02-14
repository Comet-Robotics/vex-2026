#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"

using namespace constants::outtake; 

class Outtake: public pros::MotorGroup {
    public:
        Outtake() : pros::MotorGroup(std::vector<int8_t>(OUTTAKE_PORTS.begin(), OUTTAKE_PORTS.end())) {}

        inline void forward() {this->move_voltage(MAX_OUTTAKE_SPEED);}
        inline void reverse() {this->move_voltage(-MAX_OUTTAKE_SPEED);}
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