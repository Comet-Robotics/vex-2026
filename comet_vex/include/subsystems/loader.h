#pragma once

#include "pros/adi.hpp"
#include "constants.h"

using namespace constants::loader;

class Loader : public pros::adi::DigitalOut {
    public:
        Loader() : pros::adi::DigitalOut(LOADER_PORT) {
            deactivate();
        }
        
        void activate() {
            set_value(true);
            deployed = true;
        }

        void deactivate() {
            set_value(false);
            deployed = false;
        }

        void toggle() {
            deployed = !deployed;
            set_value(deployed);
        }
        
    private:
        bool deployed = false;
};