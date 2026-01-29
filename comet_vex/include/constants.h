#pragma once

#include <array>
#include <vector>
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"

namespace constants
{
    namespace drivebase
    {
        inline std::vector<int8_t> LEFT_PORTS = {-3, 4, -5, -6};
        inline std::vector<int8_t> RIGHT_PORTS = {7, 8, 9, -10};

        inline constexpr auto CHASSIS_INTERNAL_GEARSET = pros::v5::MotorGears::blue;
        inline constexpr auto WHEEL_RADIUS = 3.25 / 2;
        inline constexpr auto DRIVETRAIN_GEAR_RATIO = 1;

        inline constexpr int8_t IMU_PORT = 1;
        inline pros::Imu IMU(IMU_PORT);
    }
}