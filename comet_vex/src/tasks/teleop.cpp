#include "tasks/teleop.h"
#include "nlohmann/json.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "flatbuffers/flatbuffers.h"
#include "messages/message_generated.h"
#include "messages/response_generated.h"
#include <fstream>

void opcontrol_initialize() {}

std::vector<uint8_t> buildCommand(int odom_pos, int motor_pos) {
    flatbuffers::FlatBufferBuilder builder(1024);
    auto motor = messages::CreateCommand(builder, odom_pos, motor_pos);
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

void opcontrol() {  
    flatbuffers::FlatBufferBuilder builder;

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Rotation odom(13);
    pros::Motor motor(16);

    odom.reset_position();

    while (true) {
        int odom_pos = odom.get_position();
        int motor_pos = motor.get_position();

        // send data
        auto buf = buildCommand(odom_pos, motor_pos);
        auto hexStr = toHex(buf);

        printf("%s\n", hexStr.c_str());

        // receive data
        std::string input;
        std::cin >> input;

        if (!input.empty()) {
            // decode hex string to byte array
            auto receivedBuf = fromHex(input);

            // verify and parse
            auto verifier = flatbuffers::Verifier(receivedBuf.data(), receivedBuf.size());
            if (!messages::VerifyCommandBuffer(verifier)) {
                return;
            }
            auto cmd = messages::GetResponse(receivedBuf.data());
            int voltage = cmd->voltage();

            // apply voltage to motor
            motor.move_voltage(voltage);
        }

        pros::delay(10);
    }
}