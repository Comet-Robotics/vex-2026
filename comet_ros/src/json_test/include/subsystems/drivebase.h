#pragma once

#include <cmath>
#include "constants.h"

class Drivebase
{
    public:
    void errorDrive(float drive, float turn)
    {
        turn /= ((drive < 0.5) ? 1.5 : 1.2);

        // int driveSign = ((drive >= 0)? 1 : -1);
        // int turnSign  = ((turn >= 0)?  1 : -1);

        // drive = driveSign * pow(drive, 2);
        // turn = turnSign * pow(turn, 2);

        int left_voltage = (drive + turn) * 12000;
        int right_voltage = (drive - turn) * 12000;

        for (int port : drivebase::LEFT_PORTS) {
            // left motors
            setVoltage(port, left_voltage);
            // printf("Left Motor %d Voltage: %d\n", port, left_voltage);
        }
        for (int port : drivebase::RIGHT_PORTS) {
            // right motors
            setVoltage(port, right_voltage);
            // printf("Right Motor %d Voltage: %d\n", port, right_voltage);
        }
    }
    
    private:
};
