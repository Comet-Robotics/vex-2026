#include "tasks/teleop.h"
#include "flatbuffers/flatbuffers.h"
#include "messages/message_generated.h"
#include "messages/response_generated.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/serial.h"
#include "pros/serial.hpp"
#include <cassert>
#include <cstdint>
#include <fstream>
#include <sstream>
extern "C" int32_t inp_buffer_read(uint32_t timeout);

uint32_t last_input = pros::millis();

std::string read_serial_nonblocking(size_t maxlen) {
    if (maxlen == 0) return std::string();

    std::string out;
    out.reserve(maxlen);

    for (size_t i = 0; i < maxlen; ++i) {
        int32_t c = inp_buffer_read(0); // 0 = non-blocking read
        if (c == -1) break; // no more data
        out.push_back(static_cast<char>(c));
        if (c == '\n') break; // stop at new line
    }

    last_input = pros::millis();

    return out;
}

std::vector<int> parse_csv_ints(const std::string& line) {
    std::vector<int> out;
    std::stringstream ss(line);
    int val;
    char comma;

    while (ss >> val) {
        out.push_back(val);
        ss >> comma;
    }
    return out;
}

// 0-3: left, 4-7: right
std::vector<int> motorNums = {-7, 8, -9, 14, 17, -18, 19, -20};
int imuPort = 1;
pros::IMU imu(imuPort);

void opcontrol_initialize() {
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    std::vector<pros::Motor> motors;
    motors.reserve(motorNums.size());
    for (int port : motorNums) {
        motors.emplace_back(port);
    }

    last_input = pros::millis();

    while (true) {
        // ----- Serial Output -----
        int lx = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int ly = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rx = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int ry = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        int imu_scaled = static_cast<int>(imu.get_heading() * 100);

        std::string line;
        
        // Controller
        line = std::to_string(lx) + "," + std::to_string(ly) + "," + std::to_string(rx) + "," + std::to_string(ry);

        // Motor velocities
        for (auto& m : motors) {
            line += "," + std::to_string(m.get_actual_velocity());
        }

        // IMU
        line += "," + std::to_string(imu_scaled);

        printf("%s\n", line.c_str());

        // ----- Serial Input -----

        std::string input = read_serial_nonblocking(512);

        auto voltages = parse_csv_ints(input);
        size_t count = std::min(voltages.size(), motors.size());

        for (size_t i = 0; i < count; ++i) {
            int v = std::clamp(voltages[i], -12000, 12000);
            motors[i].move_voltage(v);
        }

        // Stop motors if no input for 100ms
        if (pros::millis() - last_input > 100) {
            for (auto& m : motors) {
                m.move_voltage(0);
            }
        }

        pros::delay(10);
    }
}