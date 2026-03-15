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


#ifndef ROS2_CONTROL_BLUE_REACH_5__UTILS_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__UTILS_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <Eigen/Geometry>  // Include Eigen library for geometry

using namespace casadi;

namespace casadi_reach_alpha_5
{
    class Utils
    {

    public:
        uint8_t utils_id;       // Unique identifier for the utils
        Function vehicle_dynamics; // vehicle dynamics
        Function manipulator_dynamics; // manipulator dynamics
        Function manipulator_joint_lock_dynamics; //manipulator lock dynamics
        Function forward_kinematics; // forward kinematics
        Function forward_kinematics_com; // forward kinematics center of mass
        Function torque2currentMap; // forward motor utils
        Function current2torqueMap; // inverse motor utils
        Function genForces2propThrust; // functions to transform generalized forces to proper thrusts
        Function from_thrust_to_pwm; // function to transform thrust to pwm
        Function uv_Exkalman_update; // sensor fusion update for uv
        Function uv_dynamic_Exkalman_update; // sensor fusion update for uv with dynamic model
        Function manip_Exkalman_update; // sensor fusion update for uv
        Function pwm2rads; // convert pwm to rads
        Function thrust2rads; // convert thrust to rads
        Function pwm_to_thrusts; // convert pwm to thrust
        Function thruster_f2body_f;
        Function unwrap; // unwrap angle
        Function base_ext_R_to_vehicle; // reaction forces from manipulator to vehicle

        Utils() = default;
        // Constructor with member initializer list
        Utils(uint8_t dyn_id)
            : utils_id(dyn_id) {}

        void usage_cplusplus_checks(const std::string &name, const std::string &bin_name, const std::string &node_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
        Function load_casadi_fun(const std::string &name, const std::string &bin_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
    };
} // namespace casadi_reach_alpha_5
#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_