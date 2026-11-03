#pragma once
#include <casadi/casadi.hpp>
#include "ros2_control_blue_reach_5/msg/sim_manipulator_dynamics.hpp"
namespace ros2_control_blue_reach_5 {
// Shared nominal model. Plant-only changes must not modify the observer model.
        inline ros2_control_blue_reach_5::msg::SimManipulatorDynamics default_manipulator_dynamics()
        {
            auto msg = ros2_control_blue_reach_5::msg::SimManipulatorDynamics();
            msg.link_masses = {1.94000000e-01, 4.29000000e-01, 1.14999999e-01, 3.32999998e-01};
            msg.link_first_moments = {
                -0.00000000e+00, -0.00000000e+00, -0.00000000e+00, -4.29000003e-02,
                1.96649101e-02, 4.29000003e-02, 2.88077923e-03, 7.23516749e-03,
                9.16434754e-03, 2.16416476e-03, -1.19076924e-03, 8.07346553e-03};
            msg.link_inertias = {
                7.10109586e-01, 7.10109586e-01, 1.99576149e-06, -0.00000000e+00,
                -0.00000000e+00, -0.00000000e+00, 1.10178508e-01, 1.83331277e-01,
                1.04292121e-01, -3.32240937e-02, -8.30350362e-02, -3.83631263e-02,
                1.18956416e-01, 1.22363853e-01, 4.34411664e-03, -3.96112974e-04,
                -2.13904668e-02, -1.77228242e-03, 1.92510932e-02, 2.56548460e-02,
                7.17220917e-03, 1.48789886e-03, 4.53687373e-04, -1.09861913e-03};
            msg.viscous_friction = {2.39569756e+00, 2.23596482e+00, 8.19671021e-01, 3.57249665e-01};
            msg.coulomb_friction = {0.0, 0.0, 0.0, 0.0};
            msg.static_friction = {-0.0, -0.0, -0.0, -0.0};
            msg.stribeck_velocity = {0.0, 0.0, 0.0, 0.0};
            msg.gravity_vector = {0.0, 0.0, 9.81};
            msg.payload_com = {0.0, 0.0, 0.0};
            msg.payload_mass = 0.0;
            msg.payload_inertia = {0.0, 0.0, 0.0};
            msg.base_pose = {0.19, 0.0, -0.12, 3.14159, 0.0, 0.0};
            msg.world_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            msg.tip_offset_pose = {0.0, 0.0, 0.04, 0.0, 0.0, 0.0};
            msg.joint_lock_on_deadband = 0.05;
            msg.joint_lock_off_deadband = 0.10;
            msg.baumgarte_alpha = 200.0;
            msg.endeffector_mass = 300.0;
            msg.endeffector_damping = 400.0;
            msg.endeffector_stiffness = 0.0;
            return msg;
        }

        inline std::vector<casadi::DM> pack_manipulator_dynamics(
            const ros2_control_blue_reach_5::msg::SimManipulatorDynamics &msg)
        {
            std::vector<casadi::DM> values;
            values.reserve(81);
            auto append = [&values](const auto &range)
            {
                for (const double value : range)
                {
                    values.emplace_back(value);
                }
            };
            append(msg.link_masses);
            append(msg.link_first_moments);
            append(msg.link_inertias);
            append(msg.viscous_friction);
            append(msg.coulomb_friction);
            append(msg.static_friction);
            append(msg.stribeck_velocity);
            append(msg.gravity_vector);
            append(msg.payload_com);
            values.emplace_back(msg.payload_mass);
            append(msg.base_pose);
            append(msg.world_pose);
            append(msg.tip_offset_pose);
            return values;
        }
}
