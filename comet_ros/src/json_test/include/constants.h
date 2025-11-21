#pragma once

inline std::array<int, 20> voltages;

inline void setVoltage(int8_t port, int voltage) {
    if (port < 0) {
        voltage *= -1;
    }
    voltages[std::abs(port) - 1] = voltage;
}

namespace drivebase {
    inline constexpr std::array<int8_t, 4> LEFT_PORTS = {
        8,
        14,
        -7,
        -9
    };

    inline constexpr std::array<int8_t, 4> RIGHT_PORTS = {
        -18,
        -20,
        17,
        19
    };
}