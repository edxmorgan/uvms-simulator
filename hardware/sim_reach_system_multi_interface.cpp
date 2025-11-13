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
        const hardware_interface::HardwareComponentInterfaceParams &params)

    {
        if (
            hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Access the name from the HardwareInfo
        system_name = get_hardware_info().name;
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "CasADi computer from manipulator system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Testing casadi ready for operations");
        // Use CasADi's "external" to load the compiled dynamics functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "reach system");
        utils_service.current2torqueMap = utils_service.load_casadi_fun("current_to_torque_map", "libC2T.so");
        utils_service.torque2currentMap = utils_service.load_casadi_fun("torque_to_current_map", "libT2C.so");
        utils_service.manipulator_dynamics = utils_service.load_casadi_fun("Mnext_reg", "libMnext.so");
        utils_service.forward_kinematics = utils_service.load_casadi_fun("fkeval", "libFK.so");
        utils_service.forward_kinematics_com = utils_service.load_casadi_fun("fkcomeval", "libFKcom.so");
        utils_service.base_ext_R_to_vehicle = utils_service.load_casadi_fun("R_base", "libBase_ext_R_vehicle.so");
        utils_service.manip_Exkalman_update = utils_service.load_casadi_fun("ekf_update", "libmEKF_next.so");

        robot_prefix = get_hardware_info().hardware_parameters.at("prefix");

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robot_prefix : %s ", robot_prefix.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robots has %lu joints ", get_hardware_info().joints.size());

        hw_joint_struct_.reserve(get_hardware_info().joints.size());

        const std::size_t nj = get_hardware_info().joints.size();
        is_locked_.assign(nj, false); // joint lock mechanism

        //  could come from parameters instead of hard-coding
        on_db_.assign(nj, 1e-3);  // joint on deadband
        off_db_.assign(nj, 2e-3); // joint off deadband

        for (const hardware_interface::ComponentInfo &joint : get_hardware_info().joints)
        {
            std::string device_id_value = joint.parameters.at("device_id");
            double default_position = stod(joint.parameters.at("home"));

            uint8_t device_id = static_cast<uint8_t>(std::stoul(device_id_value, nullptr, 16));

            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device with id %u found", static_cast<unsigned int>(device_id));
            RCLCPP_INFO(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Device default position is %f", default_position);

            double max_effort = stod(joint.parameters.at("max_effort"));
            bool positionLimitsFlag = stoi(joint.parameters.at("has_position_limits"));
            double min_position = stod(joint.parameters.at("min_position"));
            double max_position = stod(joint.parameters.at("max_position"));
            double max_velocity = stod(joint.parameters.at("max_velocity"));
            double soft_k_position = stod(joint.parameters.at("soft_k_position"));
            double soft_k_velocity = stod(joint.parameters.at("soft_k_velocity"));
            double soft_min_position = stod(joint.parameters.at("soft_min_position"));
            double soft_max_position = stod(joint.parameters.at("soft_max_position"));
            double kt = stod(joint.parameters.at("kt"));
            double forward_I_static = stod(joint.parameters.at("forward_I_static"));
            double backward_I_static = stod(joint.parameters.at("backward_I_static"));

            Joint::State initialState(default_position);
            Joint::Limits jointLimits{.position_min = min_position, .position_max = max_position, .velocity_max = max_velocity, .effort_max = max_effort};
            Joint::SoftLimits jointSoftLimits{.position_k = soft_k_position, .velocity_k = soft_k_velocity, .position_min = soft_min_position, .position_max = soft_max_position};
            Joint::MotorInfo actuatorProp{.kt = kt, .forward_I_static = forward_I_static, .backward_I_static = backward_I_static};
            hw_joint_struct_.emplace_back(joint.name, device_id, initialState, jointLimits, positionLimitsFlag, jointSoftLimits, actuatorProp);

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

        for (const hardware_interface::ComponentInfo &gpio : get_hardware_info().gpios)
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
        constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
        try
        {
            node_frames_interface_ = std::make_shared<rclcpp::Node>(system_name + "_topics_interface");

            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_frames_interface_);
            spin_thread_ = std::thread([this]
                                       { executor_->spin(); });

            frame_transform_publisher_ = rclcpp::create_publisher<tf>(
                node_frames_interface_, DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

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
        // Initialize one EKF per joint
        const std::size_t nj = get_hardware_info().joints.size();
        x_est_list_.resize(nj);
        P_est_list_.resize(nj);
        Q_list_.resize(nj);
        R_list_.resize(nj);
        P_diag_list_.resize(nj);

        for (std::size_t i = 0; i < nj; ++i)
        {
            // state [pos, vel, acc]^T
            x_est_list_[i] = casadi::DM::zeros(3, 1);

            // covariance init
            casadi::DM P = casadi::DM::eye(3) * 0.001;
            P_est_list_[i] = P;

            // process noise
            casadi::DM Q_vec = casadi::DM::zeros(3, 1);
            Q_vec(0) = 0.001; // pos
            Q_vec(1) = 0.001; // vel
            Q_vec(2) = 0.001; // acc
            Q_list_[i] = casadi::DM::diag(Q_vec);

            // measurement noise for [pos, vel]
            casadi::DM R_vec = casadi::DM::zeros(2, 1);
            R_vec(0) = 0.01;  // position variance
            R_vec(1) = 0.005; // velocity variance
            R_list_[i] = casadi::DM::diag(R_vec);

            P_diag_list_[i] = {double(P(0, 0)), double(P(1, 1)), double(P(2, 2))};
        }

        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Initialized %zu per joint EKFs.", nj);

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
        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_FILTERED_POSITION, &hw_joint_struct_[i].current_state_.filtered_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_FILTERED_VELOCITY, &hw_joint_struct_[i].current_state_.filtered_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].current_state_.acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_ESTIMATED_ACCELERATION, &hw_joint_struct_[i].current_state_.estimated_acceleration));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].current_state_.current));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].current_state_.effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].current_state_.computed_effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT_UNCERTAINTY, &hw_joint_struct_[i].current_state_.computed_effort_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION, &hw_joint_struct_[i].current_state_.adaptive_predicted_position));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_POSITION_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_position_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_ADAPTIVE_PREDICTED_VELOCITY_UNCERTAINTY, &hw_joint_struct_[i].current_state_.adaptive_predicted_velocity_uncertainty));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_STATE_ID, &hw_joint_struct_[i].current_state_.state_id));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_joint_struct_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_joint_struct_[i].current_state_.sim_period));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[21].name, &hw_joint_struct_[i].current_state_.gravityF_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[22].name, &hw_joint_struct_[i].current_state_.gravityF_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[23].name, &hw_joint_struct_[i].current_state_.gravityF_z));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[24].name, &hw_joint_struct_[i].current_state_.gravityT_x));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[25].name, &hw_joint_struct_[i].current_state_.gravityT_y));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, get_hardware_info().joints[i].state_interfaces[26].name, &hw_joint_struct_[i].current_state_.gravityT_z));
        };

        // 0-3: payload
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[0].name, &payload_mass));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[1].name, &payload_Ixx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[2].name, &payload_Iyy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[3].name, &payload_Izz));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SimReachSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {

            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_struct_[i].command_state_.position));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_struct_[i].command_state_.velocity));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_joint_struct_[i].command_state_.acceleration));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_CURRENT, &hw_joint_struct_[i].command_state_.current));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_joint_struct_[i].command_state_.effort));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_COMPUTED_EFFORT, &hw_joint_struct_[i].command_state_.computed_effort));
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
        DM r_com_body = DM::vertcat({0.0, 0.0, 0.0});

        std::vector<DM> fkcom_args = {q, c_sample, r_com_body, base_T, world_T};
        T_com_i_ = utils_service.forward_kinematics_com(fkcom_args);
        // Time step for this cycle
        casadi::DM dt_dm(delta_seconds);

        // Run EKF for each joint, measurement y = [position, velocity]^T from current raw states
        for (std::size_t i = 0; i < get_hardware_info().joints.size(); ++i)
        {
            // Pack measurement
            casadi::DM y_k = casadi::DM::zeros(2, 1);
            y_k(0) = hw_joint_struct_[i].current_state_.position;
            y_k(1) = hw_joint_struct_[i].current_state_.velocity;

            // Bundle inputs for this joint
            std::vector<casadi::DM> ekf_inputs = {
                x_est_list_[i], // x_k
                P_est_list_[i], // P_k
                dt_dm,          // dt
                y_k,            // measurement
                Q_list_[i],     // Q
                R_list_[i]      // R
            };

            // EKF update
            std::vector<casadi::DM> ekf_out = utils_service.manip_Exkalman_update(ekf_inputs);

            // Extract results
            x_est_list_[i] = ekf_out[0];
            P_est_list_[i] = ekf_out[1];

            // Cache diagonals
            for (std::size_t d = 0; d < 3; ++d)
            {
                P_diag_list_[i][d] = double(P_est_list_[i](d, d));
            }

            // Write filtered states back to this joint
            const std::vector<double> x_est_dense = x_est_list_[i].nonzeros(); // 3 entries
            hw_joint_struct_[i].current_state_.filtered_position = x_est_dense[0];
            hw_joint_struct_[i].current_state_.filtered_velocity = x_est_dense[1];
            hw_joint_struct_[i].current_state_.estimated_acceleration = x_est_dense[2];
            hw_joint_struct_[i].current_state_.acceleration = x_est_dense[2];
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        double gravity = 0.0; // 9.81 m/s^2

        std::vector<DM> rigid_p = {
            0.194, 0.429, 0.115, 0.333, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01,
            0.01, 0.01, 0, 0, 0, 0.01, 0.01, 0.01, 0, 0, 0, 0.01, 0.01, 0.01, 0, 0, 0, 0.01,
            0.01, 0.01, 0, 0, 0,
            2, 2, 2, 2, // viscous friction coefficients
            0, 0, 0, 0, // coulomb friction coefficients
            0, 0, 0, 0, // Stribeck friction coefficients
            0, 0, 0, 0, // Stribeck velocity coefficients
            0, 0, gravity,                    // gravity
            0, 0, 0, 0,                    // payload center of mass wrt eff , payload mass
            0.19, 0, -0.12, 3.14159, 0, 0, // base to vehicle transform
            0, 0, 0, 0, 0, 0               // to world transform
        };

        // std::vector<DM> rigid_p = {
        //     1.94000000e-01, -0.00000000e+00, -0.00000000e+00, -0.00000000e+00,
        //     6.06059858e-01, 6.06059858e-01, 1.99729241e-06, -0.00000000e+00,
        //     -0.00000000e+00, -0.00000000e+00, 2.19133621e+00, 1.07223529e-01,
        //     -0.00000000e+00, 4.29000000e-01, -4.29000002e-02, 1.74149614e-02,
        //     4.29000002e-02, 1.22785586e-01, 1.79155166e-01, 8.66362218e-02,
        //     -2.93962029e-02, -7.90592890e-02, -4.03552876e-02, 2.15136762e+00,
        //     3.89012381e-02, -0.00000000e+00, 1.14999999e-01, 2.61878308e-03,
        //     6.65361179e-03, 6.49425692e-03, 1.22587991e-01, 1.28224300e-01,
        //     6.51118589e-03, -7.07869530e-04, -2.72088158e-02, -2.87950614e-03,
        //     8.17927472e-01, -3.31436986e-10, -0.00000000e+00, 3.32999999e-01,
        //     2.29509992e-03, -8.66042019e-04, 1.03324605e-02, 6.73341968e-03,
        //     1.45078863e-02, 8.65658757e-03, 1.19756029e-03, 1.74624285e-05,
        //     -1.24290035e-03, 3.50633904e-01, 1.56050941e-02, -0.00000000e+00,
        //     0, 0, 0, 0,
        //     0, 0, gravity,                 // gravity
        //     0, 0, 0, 0,                    // payload center of mass wrt eff , payload mass
        //     0.19, 0, -0.12, 3.14159, 0, 0, // base to vehicle transform
        //     0, 0, 0, 0, 0, 0               // to world transform])
        // };
        arm_state.clear();
        arm_state.reserve(10);

        arm_torques.clear();
        arm_torques.reserve(5);

        for (int j = 0; j < 5; ++j)
        {
            hw_joint_struct_[j].current_state_.sim_time = time_seconds;
            hw_joint_struct_[j].current_state_.sim_period = delta_seconds;
        };
        // First collect all positions
        for (int j = 0; j < 5; ++j)
        {
            arm_state.push_back(hw_joint_struct_[j].current_state_.position);
        };

        // Then collect all velocities
        for (int j = 0; j < 5; ++j)
        {
            arm_state.push_back(hw_joint_struct_[j].current_state_.velocity);
        };

        // Then collect all efforts, using your maps and current limiter
        for (int j = 0; j < 5; ++j)
        {
            const double tau_cmd = hw_joint_struct_[j].command_state_.effort;

            // torque -> current
            const bool pos_tau = (tau_cmd >= 0.0);
            std::vector<DM> T2C_arg = {
                DM(hw_joint_struct_[j].actuator_Properties_.kt),
                DM(pos_tau ? hw_joint_struct_[j].actuator_Properties_.forward_I_static
                           : hw_joint_struct_[j].actuator_Properties_.backward_I_static),
                DM(tau_cmd)};
            const double I_cmd = utils_service.torque2currentMap(T2C_arg).at(0).scalar();

            // clamp in current using your existing hard limiter that also enforces position limits
            const double I_safe = hw_joint_struct_[j].enforce_hard_limits(I_cmd);

            // current -> torque
            const bool pos_cur = (I_safe >= 0.0);
            std::vector<DM> C2T_arg = {
                DM(hw_joint_struct_[j].actuator_Properties_.kt),
                DM(pos_cur ? hw_joint_struct_[j].actuator_Properties_.forward_I_static
                           : hw_joint_struct_[j].actuator_Properties_.backward_I_static),
                DM(I_safe)};
            const double tau_safe = utils_service.current2torqueMap(C2T_arg).at(0).scalar();

            arm_torques.push_back(tau_safe);
        };

        // number of dynamic joints in the model
        constexpr std::size_t n_dyn = 4;

        // update hysteresis for the 4 dynamic joints
        for (std::size_t j = 0; j < n_dyn; ++j)
        {
            const double u = hw_joint_struct_[j].command_state_.effort;
            if (!is_locked_[j] && std::abs(u) < on_db_[j])
            {
                is_locked_[j] = true;
            }
            else if (is_locked_[j] && std::abs(u) > off_db_[j])
            {
                is_locked_[j] = false;
            }
        }

        // build a single column mask of length 4
        casadi::DM lock_mask = casadi::DM::zeros(static_cast<int>(n_dyn), 1);
        for (std::size_t j = 0; j < n_dyn; ++j)
        {
            lock_mask(j) = is_locked_[j] ? 1.0 : 0.0;
            // lock_mask(j) = 0.0;
        }

        // // optional log
        // RCLCPP_INFO_THROTTLE(
        //     rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
        //     *node_frames_interface_->get_clock(), 500,
        //     "efforts = [%.3e %.3e %.3e %.3e | ee %.3e], mask = [%d %d %d %d]",
        //     hw_joint_struct_[0].command_state_.effort,
        //     hw_joint_struct_[1].command_state_.effort,
        //     hw_joint_struct_[2].command_state_.effort,
        //     hw_joint_struct_[3].command_state_.effort,
        //     hw_joint_struct_[4].command_state_.effort,
        //     static_cast<int>(is_locked_[0]),
        //     static_cast<int>(is_locked_[1]),
        //     static_cast<int>(is_locked_[2]),
        //     static_cast<int>(is_locked_[3]));

        // call dynamics with the mask
        DM baumgarte_alpha = 200;
        DM endeffector_mass = 300;
        DM endeffector_damping = 400;
        DM endeffector_stiffness = 0;
        arm_simulate_argument = {arm_state, arm_torques, delta_seconds, rigid_p, endeffector_mass,
                                 endeffector_damping, endeffector_stiffness, lock_mask, baumgarte_alpha};
        arm_sim = utils_service.manipulator_dynamics(arm_simulate_argument);
        arm_next_states = arm_sim.at(0).nonzeros();

        // clamp the NEW state, then write it back
        for (int j = 0; j < 5; ++j)
        {
            double q = arm_next_states[j];
            double qd = arm_next_states[5 + j];

            const auto &lim = hw_joint_struct_[j].limits_;

            if (q < lim.position_min)
            {
                q = lim.position_min;
                if (qd < 0.0)
                    qd = 0.0;
            }
            else if (q > lim.position_max)
            {
                q = lim.position_max;
                if (qd > 0.0)
                    qd = 0.0;
            }

            hw_joint_struct_[j].current_state_.position = q;
            hw_joint_struct_[j].current_state_.velocity = qd;
        }
        // Publish TFs
        rclcpp::Time current_time = node_frames_interface_->now();
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
                t.header.stamp = current_time;
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
                t.header.stamp = current_time;
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
