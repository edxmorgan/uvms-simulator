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
#include <cmath>
#include <casadi/casadi.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>  // Include Eigen library for geometry

using namespace casadi;

namespace casadi_reach_alpha_5
{
    // Row-major continuous body-wrench process covariance (N, Nm), not an
    // observation or an injected load. Empty preserves diagonal configuration.
    inline casadi::DM wrench_process_covariance(const std::vector<double> &full,
                                                const std::vector<double> &diagonal)
    {
        if (full.empty()) return casadi::DM::diag(casadi::DM(diagonal));
        if (full.size()!=36) throw std::invalid_argument("ekf_wrench_process_covariance requires 36 row-major entries");
        Eigen::Matrix<double,6,6> matrix;
        for(int i=0;i<6;++i) for(int j=0;j<6;++j) matrix(i,j)=full[6*i+j];
        if(!matrix.allFinite() || !matrix.isApprox(matrix.transpose(),1e-10))
            throw std::invalid_argument("wrench process covariance must be finite and symmetric");
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> solver(matrix);
        if(solver.info()!=Eigen::Success || solver.eigenvalues().minCoeff()<0.)
            throw std::invalid_argument("wrench process covariance must be positive semidefinite");
        casadi::DM result=casadi::DM::zeros(6,6);
        for(int i=0;i<6;++i) for(int j=0;j<6;++j) result(i,j)=matrix(i,j);
        return result;
    }

    // The generated predictor integrates continuous process uncertainty along
    // the nominal dynamics. Bound quadrature intervals, without reusing any
    // sensor observations during the internal prediction substeps.
    inline std::vector<casadi::DM> predict_disturbance_covariance(
        const casadi::Function &predict, const casadi::DM &x, const casadi::DM &P,
        const casadi::DM &u, const casadi::DM &parameters, double dt,
        const casadi::DM &Qc)
    {
        if (!std::isfinite(dt) || dt <= 0.0)
            throw std::invalid_argument("prediction duration must be finite and positive");
        const auto steps = static_cast<std::size_t>(std::ceil(dt / 0.02));
        const double h = dt / static_cast<double>(steps);
        casadi::DM state=x, covariance=P;
        casadi::DM transition=casadi::DM::eye(x.numel());
        casadi::DM noise=casadi::DM::zeros(x.numel(),x.numel());
        for (std::size_t i=0; i<steps; ++i) {
            auto result=predict(std::vector<casadi::DM>{state,covariance,u,parameters,h,Qc});
            if (steps==1) return result;
            state=result.at(0); covariance=result.at(1);
            noise=casadi::DM::mtimes(std::vector<casadi::DM>{result.at(2),noise,result.at(2).T()})+result.at(3);
            transition=casadi::DM::mtimes(result.at(2),transition);
        }
        return {state,covariance,transition,(noise+noise.T())*0.5};
    }

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
        Function manip_Exkalman_update; // sensor fusion update for uv
        Function manip_disturbance_predict;
        Function manip_disturbance_update; // augmented arm dynamics/disturbance EKF
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
