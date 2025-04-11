// Copyright (C) 2024 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// #pragma once

#ifndef ROS2_CONTROL_BLUE_REACH_5__UNWRAP_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__UNWRAP_HPP_

#include <iostream>
#include <cmath>
#include <vector>

// Define M_PI if not defined by the math library
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace uvms
{

    // Real-time angle unwrapper class.
    class AngleUnwrapper
    {
    private:
        double prev_angle;      // Previous raw angle measurement.
        double offset;          // Cumulative offset applied.
        bool first_measurement; // Flag to check if this is the first measurement.
        const double two_pi;    // Constant for 2π.

    public:
        // Constructor: initialize values.
        AngleUnwrapper() : prev_angle(0.0),
                           offset(0.0),
                           first_measurement(true),
                           two_pi(2.0 * M_PI) {}

        // Update function takes a new angle and returns its unwrapped value.
        double update(double new_angle)
        {
            if (first_measurement)
            {
                // For the first measurement, there's no previous angle to compare.
                prev_angle = new_angle;
                first_measurement = false;
                return new_angle;
            }

            // Calculate the difference between the new measurement and the previous one.
            double diff = new_angle - prev_angle;

            // Determine how many full rotations (multiples of 2π) to adjust.
            // This uses rounding to determine the closest multiple.
            double adjustment = std::round(diff / two_pi);

            // Update the cumulative offset.
            offset -= adjustment * two_pi;

            // Calculate the unwrapped angle.
            double unwrapped = new_angle + offset;

            // Update previous measurement with the current raw angle.
            prev_angle = new_angle;
            return unwrapped;
        }
    };
} // namespace uvms
#endif // ROS2_CONTROL_BLUE_REACH_5__UNWRAP_HPP_
