#pragma once

#include <vector>
#include <cmath>

namespace MathUtils
{

    template <typename... Args>
    void scaleToUnitRange(Args &...args)
    {
        std::vector<double *> values = {&args...};
        double maxAbsValue = 0.0;

        // Find the maximum absolute value
        for (double *value : values)
        {
            maxAbsValue = fmax(maxAbsValue, fabs(*value));
        }

        // Scale values to the range [-1, 1] if maxAbsValue > 1.0
        if (maxAbsValue > 1.0)
        {
            for (double *value : values)
            {
                *value /= maxAbsValue;
            }
        }
    }

    /**
     * @brief Converts meters to inches.
     */
    inline double metersToInches(double meters)
    {
        return meters * 39.3701;
    }
}