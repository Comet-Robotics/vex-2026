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


void opcontrol_initialize() {}

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

// 0-3: left, 4-7: right
std::vector<int> motorNums = {-7, 8, -9, 14, 17, -18, 19, -20};
int imuPort = 1;

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::Imu imu(imuPort);
  std::vector<pros::Motor> motors;
  for (int i : motorNums) {
    motors.push_back(pros::Motor(i));
  }

  while (true) {
    // Byte mapping of byteBuf:
    // 1–4     : int leftX
    // 5–8     : int leftY
    // 9–12    : int rightX
    // 13–16   : int rightY
    // 17-32   : int leftMotors[4]
    // 33-48   : int rightMotors[4]
    // 49–56   : double imuHeading

    // printf("Starting loop\n");
    std::vector<int> controllerInputs = {
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),
        master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
        master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)};
    std::vector<int> motorPositions;
    for (auto m : motors) {
      motorPositions.push_back(m.get_actual_velocity());
    }

    // Combine controller inputs and motor positions into one vector
    std::vector<int> combinedData;
    combinedData.insert(combinedData.end(), controllerInputs.begin(),
                        controllerInputs.end());
    combinedData.insert(combinedData.end(), motorPositions.begin(),
                        motorPositions.end());

    // Serialize ints into bytes (4 bytes per int, big endian)
    std::vector<uint8_t> byteBuf;
    for (int val : combinedData) {
      byteBuf.push_back(static_cast<uint8_t>((val >> 24) & 0xFF));
      byteBuf.push_back(static_cast<uint8_t>((val >> 16) & 0xFF));
      byteBuf.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
      byteBuf.push_back(static_cast<uint8_t>(val & 0xFF));
    }

    // Serialize double heading from IMU (8 bytes, big endian)
    double headingDouble = imu.get_heading(); // get IMU heading
    uint8_t *p = reinterpret_cast<uint8_t *>(&headingDouble);

    // Push bytes into byteBuf in big-endian order
    for (int i = 7; i >= 0; --i) {
      byteBuf.push_back(p[i]);
    }

    // Convert to hex string
    std::ostringstream oss;
    for (auto b : byteBuf) {
      oss << std::hex << std::setw(2) << std::setfill('0')
          << static_cast<int>(b);
    }
    std::string hexStr = oss.str();

    // Print hex
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
      auto verifier =
          flatbuffers::Verifier(receivedBuf.data(), receivedBuf.size());
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
        printf("Warning: got %zu voltages but only %zu motors\n", motor_count,
               motors.size());
      }
    }

    // printf("Loop complete\n");

    pros::delay(10);
  }
}