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

#ifndef ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_

#include "ros2_control_blue_reach_5/joint.hpp"
#include "ros2_control_blue_reach_5/vehicle.hpp"

namespace uvms
{
    class State
    {
    private:
    public:
        // Store the state & commands for the robot joints
        std::vector<Joint> hw_joint_struct_;

        casadi::DM tau_covariance = 100 * casadi::DM::eye(4);
        casadi::DM x_covariance = 100 * casadi::DM::eye(8);

        struct Pose_
        {
            double position_x, position_y, position_z;
            double orientation_w, orientation_x, orientation_y, orientation_z;
        };

        Pose_ default_state_, command_state_, current_state_, async_state_;


        struct Step_condition_ {
            double time;
            double t_step;
            double je_ic_position, jd_ic_position, jc_ic_position, jb_ic_position, ja_ic_position;
            double je_ic_velocity, jd_ic_velocity, jc_ic_velocity, jb_ic_velocity, ja_ic_velocity;
            double je_ic_effort, jd_ic_effort, jc_ic_effort, jb_ic_effort, ja_ic_effort;
        };

        Step_condition_ mhe_data;

        // Store the state & commands for the robot vehicle
        blue::dynamics::Vehicle hw_vehicle_struct_;

        State() = default;
    };
} // namespace uvms
#endif // ROS2_CONTROL_BLUE_REACH_5__STATE_HPP_
