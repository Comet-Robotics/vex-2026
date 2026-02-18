#pragma once

#include "constants.h"
#include "pros/motor_group.hpp"

using namespace constants::intake;

enum class IntakeMode {
    OFF,
    FORWARD,
    REVERSE,
    UNFOLD,
    LOADER
};

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

        void setIntakeMode(IntakeMode mode) {
            this->intakeMode.store(mode);
        }

        void intakeTask() {
            while (true) {
                switch (this->intakeMode.load()) {
                    case IntakeMode::OFF:
                        this->stop();
                        break;
                    case IntakeMode::FORWARD:
                        if (isJammed()) {
                            runDeJam();
                        } else {
                            this->forward();
                        }
                        break;
                    case IntakeMode::REVERSE:
                        this->reverse();
                        break;
                    case IntakeMode::UNFOLD:
                        this->reverse();
                        pros::delay(200);
                        this->stop();
                        break;
                    case IntakeMode::LOADER:
                        break;
                }
                pros::delay(10);
            }
        }

        bool isJammed() {
            return this->get_current_draw() > JAM_CURRENT_THRESHOLD &&
                     this->get_voltage() > JAM_VOLTAGE_THRESHOLD;
        }

        void runDeJam() {
            this->reverse();
            pros::delay(100);
            this->forward();
            pros::delay(100);
        }
    private:
        std::atomic<IntakeMode> intakeMode{IntakeMode::OFF};
};

