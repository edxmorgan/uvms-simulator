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


#ifndef ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_

namespace custom_hardware_interface
{
/// Constant defining current interface
constexpr char HW_IF_CURRENT[] = "current";
/// Constant defining pwm interface
constexpr char HW_IF_PWM[] = "pwm";
/// Constant defining disable interface
constexpr char HW_IF_DISABLE[] = "disable";
/// Constant defining standby interface
constexpr char HW_IF_STANDBY[] = "standby";
/// Constant defining state counts for synchronizations purposes
constexpr char HW_IF_STATE_ID[] = "stateId";
/// Constant defining filtered_position interface
constexpr char HW_IF_FILTERED_POSITION[] = "filtered_position";
/// Constant defining filtered_velocity interface
constexpr char HW_IF_FILTERED_VELOCITY[] = "filtered_velocity";
/// Constant defining estimated_acceleration interface
constexpr char HW_IF_ESTIMATED_ACCELERATION[] = "estimated_acceleration";
/// Constant defining computed_effort interface
constexpr char HW_IF_COMPUTED_EFFORT[] = "computed_effort";
/// Constant defining computed_effort_uncertainty interface
constexpr char HW_IF_COMPUTED_EFFORT_UNCERTAINTY[] = "computed_effort_uncertainty";

/// Constant defining predicted_position interface
constexpr char HW_IF_PREDICTED_POSITION[] = "predicted_position";

/// Constant defining predicted_velocity interface
constexpr char HW_IF_PREDICTED_VELOCITY[] = "predicted_velocity";

/// Constant defining predicted_position interface
constexpr char HW_IF_PREDICTED_POSITION_UNCERTAINTY[] = "predicted_position_uncertainty";

/// Constant defining predicted_position interface
constexpr char HW_IF_PREDICTED_VELOCITY_UNCERTAINTY[] = "predicted_velocity_uncertainty";


/// Constant defining predicted_position interface
constexpr char HW_IF_ADAPTIVE_PREDICTED_POSITION[] = "adaptive_predicted_position";

/// Constant defining predicted_velocity interface
constexpr char HW_IF_ADAPTIVE_PREDICTED_VELOCITY[] = "adaptive_predicted_velocity";

/// Constant defining predicted_position interface
constexpr char HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY[] = "adaptive_predicted_position_uncertainty";

/// Constant defining predicted_position interface
constexpr char HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY[] = "adaptive_predicted_velocity_uncertainty";

/// Constant defining sim_time interface
constexpr char HW_IF_SIM_TIME[] = "sim_time";

/// Constant defining sim_period interface
constexpr char HW_IF_SIM_PERIOD[] = "sim_period";
}  // namespace hardware_interface



#endif  // ROS2_CONTROL_BLUE_REACH_5__CUSTOM_HARDWARE_INTERFACE_TYPE_VALUES_HPP_
