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


#include "ros2_control_blue_reach_5/vehicle.hpp"

namespace blue::dynamics
{
  void Vehicle::thrustSizeAllocation(const double &joint_size)
  {
    hw_thrust_structs_.reserve(joint_size);
  };

  void Vehicle::set_vehicle_name(const std::string &vehicle_name, const Pose_vel &default_state) // Corrected definition
  {
    name = vehicle_name;
    current_state_ = default_state;
    command_state_ = default_state;
    default_state_ = default_state;
    async_state_ = default_state;
  }
} // namespace blue::dynamics