#include "tasks/teleop.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "flatbuffers/flatbuffers.h"
#include "messages/message_generated.h"
#include "messages/response_generated.h"
#include <cassert>
#include <cstdint>
#include <fstream>
#include <sstream>
#include "pros/serial.h"
#include "pros/serial.hpp"
#include "pros/adi.hpp"
extern "C" int32_t inp_buffer_read(uint32_t timeout);


void opcontrol_initialize() {}

std::vector<uint8_t> buildController(float left_x, float left_y, float right_x, float right_y) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto motor = messages::CreateController(
        builder, 
        left_x, 
        left_y, 
        right_x, 
        right_y
    );
    builder.Finish(motor);

    uint8_t* buf = builder.GetBufferPointer();
    int size = builder.GetSize();
    return std::vector<uint8_t>(buf, buf + size);
}

std::string toHex(const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    for (auto b : data)
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    return oss.str();
}

std::vector<uint8_t> fromHex(const std::string& hex) {
    std::vector<uint8_t> data;
    data.reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        if (i + 1 >= hex.size()) break;  // odd length safety
        uint8_t byte = static_cast<uint8_t>(strtol(hex.substr(i, 2).c_str(), nullptr, 16));
        data.push_back(byte);
    }
    return data;
}

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
    return out;
}


void opcontrol() {  
    flatbuffers::FlatBufferBuilder builder;

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    std::vector<pros::Motor> motors;
    for (int i = 1; i <= 20; ++i) {
        motors.push_back(pros::Motor(i));
    }

    

    while (true) {
        // printf("Starting loop\n");

        // send data
        auto buf = buildController(
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0f,
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0f,
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0f,
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0f
        );
        auto hexStr = toHex(buf);

        fflush(stdout);
        printf("%s\n", hexStr.c_str());

        // receive data
        // printf("Receiving data...\n");
        std::string input = read_serial_nonblocking(2048);
        // printf("Received string: %s\n", input.c_str());

        if (!input.empty()) {
            // decode hex string to byte array
            auto receivedBuf = fromHex(input);

            // printf("Received %zu bytes\n", receivedBuf.size());

            // verify and parse
            auto verifier = flatbuffers::Verifier(receivedBuf.data(), receivedBuf.size());
            if (!messages::VerifyResponseBuffer(verifier)) {
                printf("Could not verify response buffer on VEX side\n");
                continue;
            }
            auto cmd = messages::GetResponse(receivedBuf.data());

            int motor_voltages[20];
            
            size_t motor_count = cmd->voltages()->v()->size();
            size_t apply_count = std::min(motor_count, motors.size());

            for (size_t i = 0; i < apply_count; ++i) {
                int voltage = cmd->voltages()->v()->Get(i);
                // printf("Motor %zu voltage: %d mV\n", i + 1, voltage);
                motor_voltages[i] = voltage;
            }

            for (size_t i = 0; i < apply_count; ++i) {
                motors[i].move_voltage(motor_voltages[i]);
            }

            if (motor_count != apply_count) {
                printf("Warning: got %zu voltages but only %zu motors\n",
                    motor_count, motors.size());
            }

        }

        // printf("Loop complete\n");

        pros::delay(10);
    }
}