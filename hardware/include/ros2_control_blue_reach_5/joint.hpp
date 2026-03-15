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


#ifndef ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_

#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <casadi/casadi.hpp>

class Joint
{

private:
    double soft_min_velocity = 0;
    double soft_max_velocity = 0;

public:
    std::string name;  // Name of the device or component
    uint8_t device_id; // Unique identifier for the device

    struct State
    {
        double sim_time = 0;
        double sim_period = 0;
        double position = 0;
        double filtered_position = 0;
        double err_p = 0;
        double velocity = 0;
        double filtered_velocity = 0;
        double err_v = 0;
        double estimated_acceleration = 0;
        double estimated_jerk = 0;
        double acceleration = 0;
        double current = 0;
        double effort = 0;
        double computed_effort = 0;
        double computed_effort_uncertainty = 0;
        double predicted_position = 0;
        double predicted_position_uncertainty = 0;
        double predicted_velocity = 0;
        double predicted_velocity_uncertainty = 0;

        double state_id = 0;
        casadi::DM covariance = 100 * casadi::DM::eye(4);
        std::vector<double> sigma_m = {pow(0.1, 2), pow(0.1, 2)}; // measurement noise covariance matrix
        casadi::DM Q = 0.01 * casadi::DM::eye(4);

        double adaptive_predicted_position = 0;
        double adaptive_predicted_position_uncertainty = 0;
        double adaptive_predicted_velocity = 0;
        double adaptive_predicted_velocity_uncertainty = 0;

        double gravityF_x = 0;
        double gravityF_y = 0;
        double gravityF_z = 0;

        double gravityT_x = 0;
        double gravityT_y = 0;
        double gravityT_z = 0;
        // Constructors
        State()
        {
            std::cout << "Default State constructor called.\n";
        }

        State(double pos) : position(pos), filtered_position(pos)
        {
            std::cout << "State constructor with pos called. Position set to " << pos << "\n";
        }
    };

    State default_state_{}, command_state_{}, current_state_{}, async_state_{};

    // Device capabilities and constraints
    struct Limits
    {
        double position_min = 0;
        double position_max = 0;
        double velocity_max = 0;
        double effort_max = 0;
        double phase = 0;
    };

    Limits limits_{};
    bool has_position_limits = false;

    struct SoftLimits
    {
        double position_k = 0;
        double velocity_k = 0;
        double position_min = 0;
        double position_max = 0;
    };

    SoftLimits soft_limits_{};

    struct MotorInfo
    {
        double kt = 0;
        double forward_I_static = 0;
        double backward_I_static = 0;
    };

    MotorInfo actuator_Properties_{};

    Joint() = default;
    // Constructor with member initializer list
    Joint(std::string joint_name, uint8_t joint_id, State default_state)
        : name(std::move(joint_name)),
          device_id(joint_id),
          default_state_(default_state), command_state_(default_state), current_state_(default_state), async_state_(default_state) {}

    // Constructor with member initializer list
    Joint(std::string joint_name, uint8_t joint_id, State default_state, Limits limits, bool position_limits,
          SoftLimits soft_limits, MotorInfo actuator_Properties)
        : name(std::move(joint_name)),
          device_id(joint_id),
          default_state_(default_state), command_state_(default_state), current_state_(default_state), async_state_(default_state),
          limits_(limits),
          has_position_limits(position_limits),
          soft_limits_(soft_limits),
          actuator_Properties_(actuator_Properties) {}

    /**
     * @brief Enforce position, velocity, and effort limits for a joint that is not subject to soft limits.
     *
     * @param current_effort The serial port that the manipulator is available at.
     * is considered timed out. This must be greater than 1 second; defaults to 3 seconds.
     */
    double enforce_hard_limits(const double &current_effort);
};

#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_