#pragma once

#include <cmath>
#include "constants.h"

class Drivebase
{
    public:
    void errorDrive(float drive, float turn)
    {
        drive /= 127.0;
        turn /= 127.0;

        turn /= ((drive < 0.5) ? 1.5 : 1.2);

        // int driveSign = ((drive >= 0)? 1 : -1);
        // int turnSign  = ((turn >= 0)?  1 : -1);

        // drive = driveSign * pow(drive, 2);
        // turn = turnSign * pow(turn, 2);

        int left_voltage = (drive + turn) * 12000;
        int right_voltage = (drive - turn) * 12000;

        for (size_t i = 0; i < drivebase::LEFT_PORTS.size(); i++) {
            // left motors
            int port = drivebase::LEFT_PORTS[i];
            voltages[i] = left_voltage;
        }
        for (size_t i = 0; i < drivebase::RIGHT_PORTS.size(); i++) {
            // right motors
            int port = drivebase::RIGHT_PORTS[i];
            voltages[i] = right_voltage;
        }
    }
    
    private:
};
