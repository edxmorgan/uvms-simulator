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

#include "ros2_control_blue_reach_5/sim_reach_system_multi_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{
    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "CasADi computer from manipulator system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("ReachSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
        // Use CasADi's "external" to load the compiled dynamics functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "reach system");
        utils_service.manipulator_dynamics = utils_service.load_casadi_fun("Mnext", "libMnext.so");
        utils_service.forward_kinematics = utils_service.load_casadi_fun("fkeval", "libFK.so");
        utils_service.forward_kinematics_com = utils_service.load_casadi_fun("fkcomeval", "libFKcom.so");

        robot_prefix = info_.hardware_parameters["prefix"];

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robot_prefix : %s ", robot_prefix.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robots has %lu joints ", info_.joints.size());

        hw_joint_struct_.reserve(info_.joints.size());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            std::string device_id_value = joint.parameters.at("device_id");
            double default_position = stod(joint.parameters.at("home"));

            uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

            Joint::State initialState(default_position);
            hw_joint_struct_.emplace_back(joint.name, device_id, initialState);
            // RRBotSystemMultiInterface has exactly 19 state interfaces
            // and 6 command interfaces on each joint
            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "***********************robots now has %lu joints ", hw_joint_struct_.size());
            if (joint.command_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Joint '%s' has %zu command interfaces. 6 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 27)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Joint '%s'has %zu state interfaces. 27 expected.",
                    joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // SimReachSystemMultiInterfaceHardware has exactly 4 gpio state interfaces
            if (gpio.state_interfaces.size() != 4)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 4 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // SimReachSystemMultiInterfaceHardware has exactly 0 gpio command interfaces
            if (gpio.command_interfaces.size() != 0)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu command interfaces. 0 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_configure(const rclcpp_lifecycle::State &)
    {
        constexpr auto TF_TOPIC = "/tf";
        try
        {
            node_frames_interface_ = std::make_shared<rclcpp::Node>("reach_frames_interface");

            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_frames_interface_);
            spin_thread_ = std::thread([this]
                                       { executor_->spin(); });

            frame_transform_publisher_ = rclcpp::create_publisher<tf>(
                node_frames_interface_, TF_TOPIC, rclcpp::SystemDefaultsQoS());

            realtime_frame_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(frame_transform_publisher_);

            auto &msg = realtime_frame_transform_publisher_->msg_;
            msg.transforms.resize(1); // will resize per publish
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                         "Failed TF publisher setup, %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "Successfully configured the SimReachSystemMultiInterfaceHardware system interface for serial communication!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        executor_.reset();
        node_frames_interface_.reset();
        return hardware_interface::CallbackReturn::SUCCESS;
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Shutting down the AlphaHardware system interface.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    SimReachSystemMultiInterfaceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &hw_joint_struct_[i].current_state_.filtered_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &hw_joint_struct_[i].current_state_.filtered_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].current_state_.acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &hw_joint_struct_[i].current_state_.estimated_acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].current_state_.current));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].current_state_.effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].current_state_.computed_effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT_UNCERTAINTY, &hw_joint_struct_[i].current_state_.computed_effort_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.adaptive_predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &hw_joint_struct_[i].current_state_.state_id));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_joint_struct_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_joint_struct_[i].current_state_.sim_period));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[21].name, &hw_joint_struct_[i].current_state_.gravityF_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[22].name, &hw_joint_struct_[i].current_state_.gravityF_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[23].name, &hw_joint_struct_[i].current_state_.gravityF_z));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[24].name, &hw_joint_struct_[i].current_state_.gravityT_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[25].name, &hw_joint_struct_[i].current_state_.gravityT_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, info_.joints[i].state_interfaces[26].name, &hw_joint_struct_[i].current_state_.gravityT_z));
        };

        // 0-3: payload
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &payload_mass));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &payload_Ixx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &payload_Iyy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &payload_Izz));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SimReachSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].command_state_.position));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].command_state_.velocity));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].command_state_.acceleration));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].command_state_.current));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].command_state_.effort));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].command_state_.computed_effort));
        };
        return command_interfaces;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::prepare_command_mode_switch(
        const std::vector<std::string> & /*start_interfaces*/,
        const std::vector<std::string> & /*stop_interfaces*/)
    {
        RCLCPP_INFO( // NOLINT
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "preparing command mode switch");

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Command Mode Switch successful");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Activating... please wait...");
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 0 %f", hw_joint_struct_[0].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 1  %f", hw_joint_struct_[1].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 2  %f", hw_joint_struct_[2].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "default sim joint 3  %f", hw_joint_struct_[3].current_state_.filtered_position);
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Deactivating... please wait...");

        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {

        DM q = DM::vertcat({hw_joint_struct_[0].current_state_.position,
                            hw_joint_struct_[1].current_state_.position,
                            hw_joint_struct_[2].current_state_.position,
                            hw_joint_struct_[3].current_state_.position});

        DM base_T = DM::vertcat({0.190, 0.000, -0.120, 3.141592653589793, 0.000, 0.000});
        DM world_T = DM::vertcat({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        std::vector<DM> fk_args = {q, base_T, world_T};
        T_i_ = utils_service.forward_kinematics(fk_args);

        DM c_sample = DM::vertcat({5e-12, -1e-12, 16e-12,
                                   73.563e-12, -0.091e-12, -0.734e-12,
                                   17e-12, -26e-12, 2e-12,
                                   -0.030e-12, -12e-12, -98e-12});

        std::vector<DM> fkcom_args = {q, c_sample, base_T, world_T};
        T_com_i_ = utils_service.forward_kinematics_com(fkcom_args);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        std::vector<DM> rigid_p = {0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0.194,
                                   0.429,
                                   0.115,
                                   0.333,
                                   0.01,
                                   0.01,
                                   0.01,
                                   0,
                                   0,
                                   0,
                                   0.01,
                                   0.01,
                                   0.01,
                                   0,
                                   0,
                                   0,
                                   0.01,
                                   0.01,
                                   0.01,
                                   0,
                                   0,
                                   0,
                                   0.01,
                                   0.01,
                                   0.01,
                                   0,
                                   0,
                                   0,
                                   0.002,
                                   0.002,
                                   0.002,
                                   0.002,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0,
                                   0,
                                   9.81,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.19,
                                   0.0,
                                   -0.12,
                                   3.141592653589793,
                                   0.0,
                                   0.0,
                                   0.0,
                                   1.0,
                                   0,
                                   0,
                                   0,
                                   0};
        arm_state.clear();
        arm_state.reserve(8);

        arm_torques.clear();
        arm_torques.reserve(4);

        for (int j = 0; j < 4; ++j)
        {
            hw_joint_struct_[j].current_state_.sim_time = time_seconds;
            hw_joint_struct_[j].current_state_.sim_period = delta_seconds;
        };
        // First collect all positions
        for (int j = 0; j < 4; ++j)
        {
            arm_state.push_back(hw_joint_struct_[j].current_state_.position);
        };

        // Then collect all velocities
        for (int j = 0; j < 4; ++j)
        {
            arm_state.push_back(hw_joint_struct_[j].current_state_.velocity);
        };

        for (int j = 0; j < 4; ++j)
        {
            arm_torques.push_back(0.0);
        };

        arm_simulate_argument = {arm_state, arm_torques, delta_seconds, rigid_p};
        arm_sim = utils_service.manipulator_dynamics(arm_simulate_argument);
        arm_next_states = arm_sim.at(0).nonzeros();

        hw_joint_struct_[0].current_state_.position = arm_next_states[0];
        hw_joint_struct_[1].current_state_.position = arm_next_states[1];
        hw_joint_struct_[2].current_state_.position = arm_next_states[2];
        hw_joint_struct_[3].current_state_.position = arm_next_states[3];

        hw_joint_struct_[0].current_state_.velocity = arm_next_states[4];
        hw_joint_struct_[1].current_state_.velocity = arm_next_states[5];
        hw_joint_struct_[2].current_state_.velocity = arm_next_states[6];
        hw_joint_struct_[3].current_state_.velocity = arm_next_states[7];

        if (realtime_frame_transform_publisher_ && realtime_frame_transform_publisher_->trylock())
        {
            auto &msg = realtime_frame_transform_publisher_->msg_;
            auto &transforms = msg.transforms;

            const std::string base = robot_prefix + "base_link";
            const size_t nj = T_i_.size();
            const size_t nc = T_com_i_.size();

            transforms.clear();
            transforms.resize(nj + nc);

            // Joints
            for (size_t i = 0; i < nj; ++i)
            {
                auto &t = transforms[i];
                t.header.stamp = time;
                t.header.frame_id = base;
                t.child_frame_id = robot_prefix + "joint_" + std::to_string(i);

                const auto &v_dense = T_i_[i].nonzeros(); // expected [x y z qw qx qy qz]
                if (v_dense.size() >= 7)
                {
                    t.transform.translation.x = v_dense[0];
                    t.transform.translation.y = v_dense[1];
                    t.transform.translation.z = v_dense[2];

                    tf2::Quaternion q;
                    q.setW(v_dense[3]);
                    q.setX(v_dense[4]);
                    q.setY(v_dense[5]);
                    q.setZ(v_dense[6]);
                    t.transform.rotation = tf2::toMsg(q);
                }
                else
                {
                    // fallback, zero transform on bad shape
                    t.transform.translation.x = 0.0;
                    t.transform.translation.y = 0.0;
                    t.transform.translation.z = 0.0;
                    t.transform.rotation.w = 1.0;
                    t.transform.rotation.x = 0.0;
                    t.transform.rotation.y = 0.0;
                    t.transform.rotation.z = 0.0;
                }
            }

            // COMs
            for (size_t i = 0; i < nc; ++i)
            {
                auto &t = transforms[nj + i];
                t.header.stamp = time;
                t.header.frame_id = base;
                t.child_frame_id = robot_prefix + "link_com_" + std::to_string(i);

                const auto &v_dense = T_com_i_[i].nonzeros(); // expected [x y z qw qx qy qz]
                if (v_dense.size() >= 7)
                {
                    t.transform.translation.x = v_dense[0];
                    t.transform.translation.y = v_dense[1];
                    t.transform.translation.z = v_dense[2];

                    tf2::Quaternion q;
                    q.setW(v_dense[3]);
                    q.setX(v_dense[4]);
                    q.setY(v_dense[5]);
                    q.setZ(v_dense[6]);
                    t.transform.rotation = tf2::toMsg(q);
                }
                else
                {
                    t.transform.translation.x = 0.0;
                    t.transform.translation.y = 0.0;
                    t.transform.translation.z = 0.0;
                    t.transform.rotation.w = 1.0;
                    t.transform.rotation.x = 0.0;
                    t.transform.rotation.y = 0.0;
                    t.transform.rotation.z = 0.0;
                }
            }

            realtime_frame_transform_publisher_->unlockAndPublish();
        }

        return hardware_interface::return_type::OK;
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimReachSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
