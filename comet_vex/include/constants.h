#pragma once

#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include <array>
#include <cstdint>
#include "lemlib/chassis/chassis.hpp"

namespace constants
{
    // using namespace pros;

    namespace drivebase
    {
        inline constexpr double driveExponent = 1.5;
        inline constexpr double turnExponent = 1.5;

        inline constexpr double PITCH_THRESHOLD = -7.0; // degrees

        inline constexpr bool USE_TANK = false;
        // front, back, top front, top back
        inline constexpr std::array<int8_t, 4> LEFT_PORTS = {
            -17, // front top
            -20, // back top
            18,  // front bottom
            19,  // back bottom
        };

        // front, back, top front, top back
        inline constexpr std::array<int8_t, 4> RIGHT_PORTS = {
            7,  // front top
            10, // back top
            -8, // front bottom
            -9, // back bottom
        };

        inline constexpr double DRIVETRAIN_WIDTH = 11.75; // tuned this
        inline constexpr int8_t IMU_PORT = 10;

        inline constexpr auto CHASSIS_INTERNAL_GEARSET = pros::v5::MotorGears::blue;

        // lateral PID controller
        inline const lemlib::ControllerSettings LATERAL_CONTROLLER(
            9,   // proportional gain (kP)
            1,   // integral gain (kI)
            70,  // derivative gain (kD)
            2,   // anti windup
            1,   // small error range, in inches
            100, // small error range timeout, in milliseconds
            3,   // large error range, in inches
            500, // large error range timeout, in milliseconds
            0    // maximum acceleration (slew)
        );

        // angular PID controller
        inline const lemlib::ControllerSettings ANGULAR_CONTROLLER(
            5,   // proportional gain (kP)
            0.2, // integral gain (kI)
            40,  // derivative gain (kD)
            2.5, // anti windup
            1,   // small error range, in degrees
            100, // small error range timeout, in milliseconds
            3,   // large error range, in degrees
            500, // large error range timeout, in milliseconds
            0    // maximum acceleration (slew)
        );

        // angular PID controller
        // inline const lemlib::ControllerSettings ANGULAR_CONTROLLER(
        //     6,   // proportional gain (kP)
        //     0.5, // integral gain (kI)
        //     60,  // derivative gain (kD)
        //     2.5, // anti windup
        //     0,   // small error range, in degrees
        //     0,   // small error range timeout, in milliseconds
        //     0,   // large error range, in degrees
        //     0,   // large error range timeout, in milliseconds
        //     0    // maximum acceleration (slew)
        // );

        inline pros::MotorGroup LEFT_MOTORS({LEFT_PORTS[0],
                                             LEFT_PORTS[1],
                                             LEFT_PORTS[2],
                                             LEFT_PORTS[3]},
                                            CHASSIS_INTERNAL_GEARSET);

        inline pros::MotorGroup RIGHT_MOTORS({RIGHT_PORTS[0],
                                              RIGHT_PORTS[1],
                                              RIGHT_PORTS[2],
                                              RIGHT_PORTS[3]},
                                             CHASSIS_INTERNAL_GEARSET);

        inline pros::Imu IMU(IMU_PORT);

        // drivetrain settings
        inline lemlib::Drivetrain DRIVETRAIN(
            &LEFT_MOTORS,               // left motor group
            &RIGHT_MOTORS,              // right motor group
            DRIVETRAIN_WIDTH,           // 10 inch track width
            lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
            600,                        // drivetrain rpm is 600
            2                           // horizontal drift is 2 (for now)
        );

        inline lemlib::OdomSensors SENSORS(
            nullptr, // vertical tracking wheel 1
            nullptr,
            nullptr, // horizontal tracking wheel 1
            nullptr,
            &IMU // inertial sensor
        );

        inline constexpr int DEFAULT_TIMEOUT = 3000;
        inline constexpr int DEFAULT_TIMEOUT_LONG = 5000;
    }

    namespace conveyor
    {
        inline constexpr int MAX_CONVEYOR_SPEED = 12000;
        inline constexpr int SLOW_CONVEYOR_SPEED = 9000;
        inline constexpr std::array<int8_t, 3> CONVEYOR_PORTS = {
            11, // conveyor left
            -6, // conveyor right
            -16 // intake
        };
        inline constexpr char HEIGHT_ADJUST_PORT = 'A';
    }

    namespace loader
    {
        inline constexpr char LOADER_PORT = 'G';
    }

    namespace blocker
    {
        inline constexpr char BLOCKER_PORT = 'D';
    }
}
