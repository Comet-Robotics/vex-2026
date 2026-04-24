#pragma once

#include <cstdint>
#include <array>
#include "pros/imu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"

#define EIGEN_DONT_VECTORIZE
#include "Eigen/Dense"

using namespace Eigen;

namespace constants
{
    namespace ports
    {

        // for each of these, first num is the inside motor and second num is the outside motor
        // 1
        constexpr std::array<int8_t, 2> FRONT_RIGHT_PORTS = {
            -18,
            4
        };
        // 2
        constexpr std::array<int8_t, 2> FRONT_LEFT_PORTS = {
            -12,
            11,
        };
        // 3
        constexpr std::array<int8_t, 2> BACK_LEFT_PORTS = {
            -14, // -7
            20,  // 8
        };
        // 4
        constexpr std::array<int8_t, 2> BACK_RIGHT_PORTS = {
            -6,
            7,
        }; // 3 -4 is backwards

        // constexpr std::array<int8_t, 2> FRONT_RIGHT_PORTS = {
        //     0,
        //     0,
        // };
        // constexpr std::array<int8_t, 2> FRONT_LEFT_PORTS = {
        //     0,
        //     0,
        // };
        // constexpr std::array<int8_t, 2> BACK_LEFT_PORTS = {
        //     0,
        //     0,
        // };
        // constexpr std::array<int8_t, 2> BACK_RIGHT_PORTS = {
        //     0,
        //     0,
        // };

        constexpr int8_t FRONT_RIGHT_ROTATION_SENSOR_PORT = 10;
        constexpr int8_t FRONT_LEFT_ROTATION_SENSOR_PORT = 15;
        constexpr int8_t BACK_LEFT_ROTATION_SENSOR_PORT = 16;
        constexpr int8_t BACK_RIGHT_ROTATION_SENSOR_PORT = 2;

        constexpr int8_t IMU_PORT = 1;
        constexpr int8_t DISTANCE_PORT = 13;
    }

    namespace controls
    {
        // subsystem controls
        constexpr auto INTAKE_LOADER = pros::E_CONTROLLER_DIGITAL_L1;
        constexpr auto INTAKE_FLOOR = pros::E_CONTROLLER_DIGITAL_L2;
        constexpr auto SCORE = pros::E_CONTROLLER_DIGITAL_R2;
        constexpr auto OUTTAKE_LOW = pros::E_CONTROLLER_DIGITAL_A;
        constexpr auto ADJUST_DOWN = pros::E_CONTROLLER_DIGITAL_R1;
        constexpr auto PARK = pros::E_CONTROLLER_DIGITAL_LEFT;
        constexpr auto UNPARK = pros::E_CONTROLLER_DIGITAL_RIGHT;
        constexpr auto WINGS_DOWN = pros::E_CONTROLLER_DIGITAL_Y;
        constexpr auto WINGS_UP = pros::E_CONTROLLER_DIGITAL_X;

        // swerve controls
        constexpr auto RESET_POSE = pros::E_CONTROLLER_DIGITAL_UP;
        constexpr auto X_WHEELS = pros::E_CONTROLLER_DIGITAL_B;
        constexpr auto HEADING_HOLD_TOGGLE = pros::E_CONTROLLER_DIGITAL_DOWN;
    }

    namespace drivetrain
    {
        constexpr pros::MotorGears CHASSIS_INTERNAL_GEARSET = pros::MotorGears::blue;

        constexpr double MAX_LINEAR_SPEED = 77.0;   // inches per second
        constexpr double MAX_ANGULAR_SPEED = 550.0; // degrees per second

        constexpr double WHEEL_DIAMETER = 2.75;      // inches
        constexpr double GEAR_RATIO = 544.0 / 555.0; // output (wheel) speed / input (motor) speed

        constexpr double ROTATION_FACTOR = (20 * 360.0) / (27669 + 27854); // Number of rotations * 360 degrees / difference in encoder counts
        // how to tune: go forward a known distance while keeping the module at a fixed angle, and set factor to (actual distance traveled) / (calculated distance)
        constexpr double LINEAR_FACTOR = (24.0 / 28.0) * (24.0 / 24.65);
        constexpr double TRACK_LENGTH = 8;     // distance between front and back wheels
        constexpr double TRACK_WIDTH = 10.125; // distance between left and right wheels

        inline pros::Imu IMU(ports::IMU_PORT);
        inline pros::Distance DISTANCE(ports::DISTANCE_PORT);

        inline constexpr double distOffsetX = TRACK_WIDTH / 2 - 1.77;
        inline constexpr double distOffsetY = TRACK_LENGTH / 2 - 1;
        inline constexpr double wallY = 0; // y coordinate of wall

        constexpr std::array<std::array<double, 2>, 4> wheelPositions = {
            std::array<double, 2>{TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0}, // Front Right
            std::array<double, 2>{TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0},  // Front Left
            std::array<double, 2>{-TRACK_LENGTH / 2.0, TRACK_WIDTH / 2.0}, // Back Left
            std::array<double, 2>{-TRACK_LENGTH / 2.0, -TRACK_WIDTH / 2.0} // Back Right
        };

        // conversion matrix for kinematics
        inline Matrix<double, 8, 3> initializeConversionMatrix()
        {
            Matrix<double, 8, 3> matrix;
            for (int i = 0; i < 4; i++)
            {
                double x = wheelPositions[i][0];
                double y = wheelPositions[i][1];
                matrix.row(i * 2) = Vector3d(1, 0, -y);
                matrix.row(i * 2 + 1) = Vector3d(0, 1, x);
            }
            return matrix;
        }

        inline Matrix<double, 8, 3> CONVERSION_MATRIX = initializeConversionMatrix();

        constexpr double kP = 0.02;
        constexpr double kI = 0.0;
        constexpr double kD = 0.001;

        constexpr std::array<double, 3> FRONT_LEFT_PID = {
            kP,
            kI,
            kD,
        };
        constexpr std::array<double, 3> FRONT_RIGHT_PID = {
            kP,
            kI,
            kD,
        };
        constexpr std::array<double, 3> BACK_LEFT_PID = {
            kP,
            kI,
            kD,
        };
        constexpr std::array<double, 3> BACK_RIGHT_PID = {
            kP,
            kI,
            kD,
        };

        constexpr std::array<double, 3> X_PID = {
            12.0,
            0.0,
            0.0,
        };
        constexpr std::array<double, 3> Y_PID = {
            12.0,
            0.0,
            0.0,
        };
        constexpr std::array<double, 3> THETA_PID = {
            18.0,
            0.0,
            0.0,
        };

        constexpr std::array<double, 3> HEADING_HOLD_PID = {
            0.04,
            0.0,
            0.0,
        };

        constexpr double DEADZONE_THRESHOLD = 0.1; // for controller inputs

        // FEATURE FLAGS
        constexpr bool COSINE_SCALING = true;        // whether to scale speed by cosine of angle delta
        constexpr bool STAY_AT_ORIENTATION = true;   // whether to maintain pod orientation when not commanded to rotate
        constexpr bool FIELD_CENTRIC_DEFAULT = true; // whether to use field-centric controls by default
        constexpr bool USE_ROTATION_SENSORS = true;  // whether to use separate rotation sensors for module angle
        constexpr bool HEADING_HOLD = true;          // whether to maintain heading when not commanded to rotate
    }

    namespace autonomous
    {
        constexpr double TIME_TOLERANCE = 0.05; // seconds
    }

    namespace conveyor
    {
        inline constexpr int MAX_CONVEYOR_SPEED = 12000;
        inline constexpr int SLOW_CONVEYOR_SPEED = 7500;
        inline constexpr int SLOW_REVERSE_SPEED = 7500;
        inline constexpr std::array<int8_t, 3> CONVEYOR_PORTS = {
            -8,   // intake
            -19,   // conveyor
            9, // outtake
        };
        inline constexpr char HEIGHT_ADJUST_PORT = 'D';
    }

    namespace loader
    {
        inline constexpr char LOADER_PORT = 'C';
    }

    namespace blocker
    {
        inline constexpr char BLOCKER_PORT = 'B';
    }

    namespace wings
    {
        inline constexpr char WINGS_PORT = 'A';
    }

    namespace park
    {
        inline constexpr std::array<int8_t, 2> PARK_PORTS = {
            -3,   // left
            17, // right (reversed)
        };
        inline constexpr int MAX_PARK_SPEED = 12000;
        inline constexpr int SETUP_POSITION = 50; // degrees to rotate for setup
    }

    constexpr int MOTOR_TEMPERATURE_THRESHOLD = 55; // degrees Celsius
    constexpr int TELEOP_POLL_TIME = 20;            // milliseconds
}