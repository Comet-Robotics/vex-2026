#pragma once

inline std::array<int8_t, 20> voltages;

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