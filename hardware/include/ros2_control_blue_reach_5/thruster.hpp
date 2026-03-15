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

#ifndef ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_

#include <string>
#include <algorithm>
#include "rcl_interfaces/msg/parameter.hpp"

class Thruster
{

public:
    std::string name; // Name of the device or component
    rcl_interfaces::msg::Parameter param;
    int channel;
    int neutral_pwm = 1500;
    int rc_direction = 1;

    struct State
    {
        double sim_time = 0;
        double sim_period = 0;
        double position = 0;
        double velocity = 0;
        double command_pwm = 1500;
        double rc_pwm = 0;
        double acceleration = 0;
        double effort = 0;
    };

    State default_state_{}, command_state_{}, current_state_{}, async_state_{};

    Thruster() = default;
    // Constructor with member initializer list
    Thruster(std::string joint_name, State default_state)
        : name(std::move(joint_name)),
          default_state_(default_state) {}

    // Constructor with member initializer list for bluerov mavros
    Thruster(std::string joint_name, rcl_interfaces::msg::Parameter param, int channel, int neutral_pwm, int rc_direction, State default_state)
        : name(std::move(joint_name)),
          param(std::move(param)),
          channel(std::move(channel)),
          neutral_pwm(std::move(neutral_pwm)),
          rc_direction(std::move(rc_direction)),
          default_state_(default_state) {}
    void calcAcceleration(const double &prev_velocity_, const double &period_seconds);
};

#endif // ROS2_CONTROL_BLUE_REACH_5__THRUSTER_HPP_