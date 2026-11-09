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
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/arm_actuator_model.hpp"
#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"
#include "ros2_control_blue_reach_5/arm_time_integration.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{
    namespace
    {
        constexpr double kTorqueCurrentZeroEpsilon = 1.0e-9;


        double gravity_magnitude(const std::array<double, 3> &gravity_vector)
        {
            return std::sqrt(
                gravity_vector[0] * gravity_vector[0] +
                gravity_vector[1] * gravity_vector[1] +
                gravity_vector[2] * gravity_vector[2]);
        }

        double dense_vector_value(const casadi::DM &matrix, const std::size_t index)
        {
            const std::size_t rows = static_cast<std::size_t>(matrix.size1());
            const std::size_t cols = static_cast<std::size_t>(matrix.size2());
            if (rows == 1 && index < cols)
            {
                return static_cast<double>(matrix(0, static_cast<int>(index)).scalar());
            }
            if (cols == 1 && index < rows)
            {
                return static_cast<double>(matrix(static_cast<int>(index), 0).scalar());
            }
            throw std::out_of_range("CasADi output is not a vector with the requested index");
        }

        std::vector<double> dense_vector(const casadi::DM &matrix, const std::size_t expected_size)
        {
            std::vector<double> values;
            values.reserve(expected_size);
            for (std::size_t i = 0; i < expected_size; ++i)
            {
                values.push_back(dense_vector_value(matrix, i));
            }
            return values;
        }
    } // namespace

    SimReachSystemMultiInterfaceHardware::~SimReachSystemMultiInterfaceHardware()
    {
        stop_ros_interfaces();
    }

    void SimReachSystemMultiInterfaceHardware::reset_joint_estimators()
    {
        arm_x_est_ = casadi::DM::zeros(12, 1);
        arm_P_est_ = casadi::DM::eye(12) * 0.001;
        arm_Q_ = casadi::DM::zeros(12, 12);
        arm_R_ = casadi::DM::zeros(8, 8);
        for (int i = 0; i < 4; ++i)
        {
            arm_P_est_(8 + i, 8 + i) = 1.0;
            arm_Q_(i, i) = 1e-8;
            arm_Q_(4 + i, 4 + i) = 1e-6;
            // Continuous torque random-walk density (Nm^2/s). Validated with
            // the configured encoder R, noisy contact/release and load steps.
            // The former 0.1 followed encoder noise as rapidly varying torque.
            arm_Q_(8 + i, 8 + i) = 0.001;
            arm_R_(i, i) = 1e-6;
            arm_R_(4 + i, 4 + i) = 1e-5;
        }
        estimated_disturbance_.fill(0.0);
        disturbance_variance_.fill(1.0);
        disturbance_covariance_.fill(0.0);
        for (int i=0;i<4;++i) disturbance_covariance_[4*i+i]=1.0;
        estimator_applied_input_ = casadi::DM::zeros(4, 1);
        estimator_completed_lock_mask_ = casadi::DM::zeros(4, 1);
        estimator_completed_contact_ = casadi::DM::zeros(4, 1);
        estimator_lower_limits_ = casadi::DM::zeros(4, 1);
        estimator_upper_limits_ = casadi::DM::zeros(4, 1);
        for (std::size_t i=0; i<4 && i<hw_joint_struct_.size(); ++i)
        {
            estimator_lower_limits_(i) = hw_joint_struct_[i].limits_.position_min;
            estimator_upper_limits_(i) = hw_joint_struct_[i].limits_.position_max;
        }
        estimator_joint_at_limit_.fill(0.0);
        estimator_disturbance_observable_.fill(0.0);
        estimator_completed_lock_alpha_ = 200.0;
        estimator_joint_locked_.fill(0.0);
        estimator_step_ = 0.0;
        estimator_valid_ = 0.0;
        estimator_completed_dt_ = 0.0;
        // Fixed dory_alpha nominal model shared with the controller and default
        // plant. Subsequent plant-only changes remain estimation targets.
        auto nominal = default_manipulator_dynamics();
        estimator_nominal_parameters_ = casadi::DM::vertcat(pack_manipulator_dynamics(nominal));
        const auto parameters = dense_vector(estimator_nominal_parameters_, 81);
        std::copy(parameters.begin(), parameters.end(), estimator_model_readback_.begin());

        const std::size_t nj = get_hardware_info().joints.size();
        x_est_list_.resize(nj);
        P_est_list_.resize(nj);
        Q_list_.resize(nj);
        R_list_.resize(nj);
        P_diag_list_.resize(nj);

        for (std::size_t i = 0; i < nj; ++i)
        {
            x_est_list_[i] = casadi::DM::zeros(3, 1);

            casadi::DM P = casadi::DM::eye(3) * 0.001;
            P_est_list_[i] = P;

            casadi::DM Q = casadi::DM::zeros(3, 3);
            Q(0, 0) = 0.001;
            Q(1, 1) = 0.001;
            Q(2, 2) = 0.001;
            Q_list_[i] = Q;

            casadi::DM R = casadi::DM::zeros(2, 2);
            R(0, 0) = 0.01;
            R(1, 1) = 0.005;
            R_list_[i] = R;

            P_diag_list_[i] = {0.001, 0.001, 0.001};
        }
    }

    void SimReachSystemMultiInterfaceHardware::seed_joint_estimator_from_state(
        std::size_t index, const Joint::State &state)
    {
        if (index >= x_est_list_.size())
        {
            return;
        }
        x_est_list_[index] = casadi::DM::zeros(3, 1);
        x_est_list_[index](0) = state.position;
        x_est_list_[index](1) = state.velocity;
        x_est_list_[index](2) = state.acceleration;
        if (index < 4)
        {
            arm_x_est_(static_cast<int>(index)) = state.position;
            arm_x_est_(4 + static_cast<int>(index)) = state.velocity;
        }
    }

    void SimReachSystemMultiInterfaceHardware::update_joint_locks()
    {
        // A startup mode, independent of later plant profile/reset requests.
        // The resulting mask is shared by plant integration and its EKF step.
        if (!arm_joint_locks_enabled_)
        {
            std::fill(is_locked_.begin(), is_locked_.end(), false);
            return;
        }
        for (std::size_t j = 0; j < 4; ++j)
        {
            const double torque = double(arm_torques[j]);
            if (!is_locked_[j] && std::abs(torque) < on_db_[j])
                is_locked_[j] = true;
            else if (is_locked_[j] && std::abs(torque) > off_db_[j])
                is_locked_[j] = false;
        }
    }

    void SimReachSystemMultiInterfaceHardware::apply_motor_lock()
    {
        // Decide hysteresis from the limited request before switching motors off.
        // Never use the previous zeroed feedback to decide whether to unlock.
        update_joint_locks();
        for (std::size_t j = 0; j < 4; ++j)
        {
            if (!is_locked_[j]) continue;
            arm_torques[j] = 0.0;
            auto &state = hw_joint_struct_[j].current_state_;
            state.current = 0.0;
            state.effort = 0.0;
            state.computed_effort_uncertainty = state.computed_effort;
        }
    }

    void SimReachSystemMultiInterfaceHardware::reset_joint_simulation_state()
    {
        if (disturbance_receiver_) disturbance_receiver_->reset();
        sim_step_ = 0.0;
        sim_generation_ += 1.0;
        const std::size_t joint_count = hw_joint_struct_.size();
        for (auto &joint : hw_joint_struct_)
        {
            joint.current_state_ = joint.default_state_;
            joint.async_state_ = joint.default_state_;
            joint.command_state_ = joint.default_state_;

            joint.command_state_.velocity = 0.0;
            joint.command_state_.acceleration = 0.0;
            joint.command_state_.current = 0.0;
            joint.command_state_.effort = 0.0;
            joint.command_state_.computed_effort = 0.0;

            joint.current_state_.velocity = 0.0;
            joint.current_state_.acceleration = 0.0;
            joint.current_state_.estimated_acceleration = 0.0;
            joint.current_state_.current = 0.0;
            joint.current_state_.effort = 0.0;
            joint.current_state_.computed_effort = 0.0;
            joint.current_state_.computed_effort_uncertainty = 0.0;
            joint.current_state_.predicted_position = 0.0;
            joint.current_state_.predicted_position_uncertainty = 0.0;
            joint.current_state_.predicted_velocity = 0.0;
            joint.current_state_.predicted_velocity_uncertainty = 0.0;
            joint.current_state_.adaptive_predicted_position = 0.0;
            joint.current_state_.adaptive_predicted_position_uncertainty = 0.0;
            joint.current_state_.adaptive_predicted_velocity = 0.0;
            joint.current_state_.adaptive_predicted_velocity_uncertainty = 0.0;
            joint.current_state_.state_id = 0.0;
            joint.current_state_.sim_time = 0.0;
            joint.current_state_.sim_period = 0.0;
            joint.current_state_.gravityF_x = 0.0;
            joint.current_state_.gravityF_y = 0.0;
            joint.current_state_.gravityF_z = 0.0;
            joint.current_state_.gravityT_x = 0.0;
            joint.current_state_.gravityT_y = 0.0;
            joint.current_state_.gravityT_z = 0.0;
        }

        std::fill(is_locked_.begin(), is_locked_.end(), false);
        control_power_ = 0.0;
        control_energy_ = 0.0;
        payload_mass = 0.0;
        payload_Ixx = 0.0;
        payload_Iyy = 0.0;
        payload_Izz = 0.0;
        if (manipulator_parameters_.size() >= 59)
            gravity_ = std::sqrt(
                std::pow(double(manipulator_parameters_[56]), 2) +
                std::pow(double(manipulator_parameters_[57]), 2) +
                std::pow(double(manipulator_parameters_[58]), 2));
        delta_seconds = 0.0;
        time_seconds = 0.0;
        commands_held_ = true;
        reset_joint_estimators();
        for (std::size_t i = 0; i < hw_joint_struct_.size(); ++i)
        {
            seed_joint_estimator_from_state(i, hw_joint_struct_[i].current_state_);
        }
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "[%s] reset simulated manipulator state for %zu joints; commands held until release",
            robot_prefix.c_str(), joint_count);
    }

    void SimReachSystemMultiInterfaceHardware::reset_joint_simulation_state(
        const ros2_control_blue_reach_5::srv::ResetSimUvms::Request &request)
    {
        reset_joint_simulation_state();

        if (request.use_manipulator_state)
        {
            const std::size_t joint_count = std::min<std::size_t>(hw_joint_struct_.size(), 5);
            for (std::size_t i = 0; i < joint_count; ++i)
            {
                auto &joint = hw_joint_struct_[i];
                const double position = request.manipulator_position[i];
                const double velocity = request.manipulator_velocity[i];

                joint.current_state_.position = position;
                joint.current_state_.filtered_position = position;
                joint.current_state_.predicted_position = position;
                joint.current_state_.adaptive_predicted_position = position;
                joint.current_state_.velocity = velocity;
                joint.current_state_.filtered_velocity = velocity;
                joint.current_state_.predicted_velocity = velocity;
                joint.current_state_.adaptive_predicted_velocity = velocity;

                joint.async_state_.position = position;
                joint.async_state_.filtered_position = position;
                joint.async_state_.velocity = velocity;
                joint.async_state_.filtered_velocity = velocity;

                joint.command_state_.position = position;
                joint.command_state_.filtered_position = position;
                joint.command_state_.velocity = 0.0;
                joint.command_state_.acceleration = 0.0;
                joint.command_state_.current = 0.0;
                joint.command_state_.effort = 0.0;
                joint.command_state_.computed_effort = 0.0;

                seed_joint_estimator_from_state(i, joint.current_state_);
            }
        }

        if (request.set_manipulator_dynamics)
        {
            const auto &dynamics = request.manipulator_dynamics;
            use_coupled_dynamics_ = request.use_coupled_dynamics;
            manipulator_parameters_ = pack_manipulator_dynamics(dynamics);
            joint_lock_on_deadband_ = dynamics.joint_lock_on_deadband;
            joint_lock_off_deadband_ = dynamics.joint_lock_off_deadband;
            baumgarte_alpha_ = dynamics.baumgarte_alpha;
            endeffector_mass_ = dynamics.endeffector_mass;
            endeffector_damping_ = dynamics.endeffector_damping;
            endeffector_stiffness_ = dynamics.endeffector_stiffness;
            std::fill(on_db_.begin(), on_db_.end(), joint_lock_on_deadband_);
            std::fill(off_db_.begin(), off_db_.end(), joint_lock_off_deadband_);
            gravity_ = gravity_magnitude(dynamics.gravity_vector);
            payload_mass = dynamics.payload_mass;
            payload_Ixx = dynamics.payload_inertia[0];
            payload_Iyy = dynamics.payload_inertia[1];
            payload_Izz = dynamics.payload_inertia[2];
        }
        commands_held_ = request.hold_commands;

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
            "[%s] reset simulated manipulator with requested state; hold=%s gravity=%.6f mass=%.3f",
            robot_prefix.c_str(),
            commands_held_ ? "true" : "false",
            gravity_,
            payload_mass);
    }

    void SimReachSystemMultiInterfaceHardware::apply_pending_reset_request()
    {
        std::optional<ros2_control_blue_reach_5::srv::ResetSimUvms::Request> request;
        std::uint64_t sequence = 0;
        {
            std::lock_guard<std::mutex> request_lock(reset_request_mutex_);
            if (!pending_reset_request_)
            {
                return;
            }
            request = pending_reset_request_;
            pending_reset_request_.reset();
            sequence = reset_request_sequence_;
        }

        try
        {
            reset_joint_simulation_state(*request);
            {
                std::lock_guard<std::mutex> request_lock(reset_request_mutex_);
                reset_failure_message_.clear();
                reset_applied_sequence_ = sequence;
            }
        }
        catch (const std::exception &e)
        {
            std::lock_guard<std::mutex> request_lock(reset_request_mutex_);
            reset_failure_message_ = e.what();
            reset_failed_sequence_ = sequence;
            reset_applied_sequence_ = sequence;
        }
        catch (...)
        {
            std::lock_guard<std::mutex> request_lock(reset_request_mutex_);
            reset_failure_message_ = "unknown exception while applying manipulator reset";
            reset_failed_sequence_ = sequence;
            reset_applied_sequence_ = sequence;
        }
        reset_applied_cv_.notify_all();
    }

    void SimReachSystemMultiInterfaceHardware::stop_ros_interfaces() noexcept
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        reset_service_.reset();
        release_service_.reset();
        dynamics_service_.reset();
        dynamics_readback_service_.reset();
        disturbance_receiver_.reset();
        executor_.reset();
        node_frames_interface_.reset();
    }

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
        utils_service.manipulator_dynamics = utils_service.load_casadi_fun("arm_plant_next", "libmDisturbanceEKF_next.so");
        utils_service.manipulator_joint_lock_dynamics = utils_service.load_casadi_fun("hard_lock_hysteresis", "libLock_f.so");
        utils_service.forward_kinematics = utils_service.load_casadi_fun("fkeval", "libFK.so");
        utils_service.forward_kinematics_com = utils_service.load_casadi_fun("fkcomeval", "libFKcom.so");
        utils_service.base_ext_R_to_vehicle = utils_service.load_casadi_fun("R_base", "libBase_ext_R_vehicle.so");
        utils_service.manip_Exkalman_update = utils_service.load_casadi_fun("ekf_update", "libmEKF_next.so");
        utils_service.manip_disturbance_predict = utils_service.load_casadi_fun(
            "arm_disturbance_ekf_predict_locked", "libmDisturbanceEKF_next.so");
        utils_service.manip_disturbance_update = utils_service.load_casadi_fun(
            "arm_disturbance_ekf_update_limited", "libmDisturbanceEKF_next.so");
        

        robot_prefix = get_hardware_info().hardware_parameters.at("prefix");
        const auto lock_option = get_hardware_info().hardware_parameters.find("arm_joint_locks");
        if (lock_option != get_hardware_info().hardware_parameters.end())
        {
            const auto &value = lock_option->second;
            if (value != "true" && value != "false" && value != "True" && value != "False")
            {
                RCLCPP_ERROR(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                             "arm_joint_locks must be true or false");
                return hardware_interface::CallbackReturn::ERROR;
            }
            arm_joint_locks_enabled_ = value == "true" || value == "True";
        }
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Artificial arm joint locks: %s", arm_joint_locks_enabled_ ? "enabled" : "disabled");
        payload_mass = 0.0;
        payload_Ixx = 0.0;
        payload_Iyy = 0.0;
        payload_Izz = 0.0;
        const auto dynamics = default_manipulator_dynamics();
        gravity_ = gravity_magnitude(dynamics.gravity_vector);
        manipulator_parameters_ = pack_manipulator_dynamics(dynamics);
        joint_lock_on_deadband_ = dynamics.joint_lock_on_deadband;
        joint_lock_off_deadband_ = dynamics.joint_lock_off_deadband;
        baumgarte_alpha_ = dynamics.baumgarte_alpha;
        endeffector_mass_ = dynamics.endeffector_mass;
        endeffector_damping_ = dynamics.endeffector_damping;
        endeffector_stiffness_ = dynamics.endeffector_stiffness;

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robot_prefix : %s ", robot_prefix.c_str());

        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "robots has %lu joints ", get_hardware_info().joints.size());

        hw_joint_struct_.reserve(get_hardware_info().joints.size());

        const std::size_t nj = get_hardware_info().joints.size();
        is_locked_.assign(nj, false); // joint lock mechanism

        on_db_.assign(nj, joint_lock_on_deadband_);
        off_db_.assign(nj, joint_lock_off_deadband_);

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

            RCLCPP_DEBUG(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                "[%s] simulated manipulator joints configured: %lu",
                robot_prefix.c_str(), hw_joint_struct_.size());
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
            // Existing metadata plus augmented EKF outputs and nominal-model readback.
            if (gpio.state_interfaces.size() != 128)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 128 expected.", gpio.name.c_str(),
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
            disturbance_receiver_ = std::make_shared<DisturbanceReceiver>(
                node_frames_interface_, robot_prefix, "arm");

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
        reset_joint_estimators();

        auto reset_state_callback =
            [this](
                const std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Request> request,
                std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Response> response)
        {
            if (request->set_manipulator_dynamics &&
                (request->manipulator_dynamics.payload_mass < 0.0 ||
                 request->manipulator_dynamics.payload_inertia[0] < 0.0 ||
                 request->manipulator_dynamics.payload_inertia[1] < 0.0 ||
                 request->manipulator_dynamics.payload_inertia[2] < 0.0))
            {
                response->success = false;
                response->message = "payload values must be non-negative";
                return;
            }

            std::unique_lock<std::mutex> request_lock(reset_request_mutex_);
            pending_reset_request_ = *request;
            const auto sequence = ++reset_request_sequence_;
            request_lock.unlock();
            reset_applied_cv_.notify_all();

            request_lock.lock();
            const bool applied = reset_applied_cv_.wait_for(
                request_lock,
                std::chrono::seconds(2),
                [this, sequence]()
                {
                    return reset_applied_sequence_ >= sequence;
                });
            if (!applied)
            {
                response->success = false;
                response->message = "timeout applying simulated manipulator reset";
                return;
            }
            if (reset_failed_sequence_ == sequence)
            {
                response->success = false;
                response->message = reset_failure_message_;
                return;
            }
            response->success = true;
            response->message = "reset simulated manipulator with requested state";
        };

        reset_service_ = node_frames_interface_->create_service<ros2_control_blue_reach_5::srv::ResetSimUvms>(
            "/" + robot_prefix + "reset_sim_manipulator",
            reset_state_callback);

        release_service_ = node_frames_interface_->create_service<std_srvs::srv::Trigger>(
            "/" + robot_prefix + "release_sim_manipulator",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                std::lock_guard<std::mutex> lock(simulation_state_mutex_);
                commands_held_ = false;
                RCLCPP_INFO(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "[%s] released simulated manipulator commands",
                    robot_prefix.c_str());
                response->success = true;
                response->message = "released simulated manipulator commands";
            });

        dynamics_service_ = node_frames_interface_->create_service<ros2_control_blue_reach_5::srv::SetSimDynamics>(
            "/" + robot_prefix + "set_sim_manipulator_dynamics",
            [this](
                const std::shared_ptr<ros2_control_blue_reach_5::srv::SetSimDynamics::Request> request,
                std::shared_ptr<ros2_control_blue_reach_5::srv::SetSimDynamics::Response> response)
            {
                if (!request->set_manipulator_dynamics)
                {
                    response->success = false;
                    response->message = "set_manipulator_dynamics must be true";
                    return;
                }
                const auto &dynamics = request->manipulator;
                if (dynamics.payload_mass < 0.0 ||
                    dynamics.payload_inertia[0] < 0.0 ||
                    dynamics.payload_inertia[1] < 0.0 ||
                    dynamics.payload_inertia[2] < 0.0)
                {
                    response->success = false;
                    response->message = "payload values must be non-negative";
                    return;
                }

                std::lock_guard<std::mutex> lock(simulation_state_mutex_);
                use_coupled_dynamics_ = request->use_coupled_dynamics;
                manipulator_parameters_ = pack_manipulator_dynamics(dynamics);
                joint_lock_on_deadband_ = dynamics.joint_lock_on_deadband;
                joint_lock_off_deadband_ = dynamics.joint_lock_off_deadband;
                baumgarte_alpha_ = dynamics.baumgarte_alpha;
                endeffector_mass_ = dynamics.endeffector_mass;
                endeffector_damping_ = dynamics.endeffector_damping;
                endeffector_stiffness_ = dynamics.endeffector_stiffness;
                std::fill(on_db_.begin(), on_db_.end(), joint_lock_on_deadband_);
                std::fill(off_db_.begin(), off_db_.end(), joint_lock_off_deadband_);
                gravity_ = gravity_magnitude(dynamics.gravity_vector);
                payload_mass = dynamics.payload_mass;
                payload_Ixx = dynamics.payload_inertia[0];
                payload_Iyy = dynamics.payload_inertia[1];
                payload_Izz = dynamics.payload_inertia[2];

                RCLCPP_INFO(
                    rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "[%s] updated sim dynamics gravity=%.6f mass=%.3f ixx=%.6f iyy=%.6f izz=%.6f use_coupled_dynamics=%s",
                    robot_prefix.c_str(), gravity_, payload_mass, payload_Ixx, payload_Iyy, payload_Izz,
                    use_coupled_dynamics_ ? "true" : "false");
                response->success = true;
                response->message = "updated simulated dynamics";
            });

        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                    "Initialized 12-state arm disturbance EKF and gripper acceleration filter.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimReachSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
    {
        stop_ros_interfaces();
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

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "sim_step", &sim_step_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "sim_generation", &sim_generation_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "estimator_step", &estimator_step_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "estimator_valid", &estimator_valid_));
        const std::array<std::string, 4> axes{"e", "d", "c", "b"};
        for (std::size_t i = 0; i < axes.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimated_disturbance.torque." + axes[i],
                &estimated_disturbance_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimated_disturbance.variance.torque." + axes[i],
                &disturbance_variance_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimator_joint_locked." + axes[i],
                &estimator_joint_locked_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimator_joint_at_limit." + axes[i],
                &estimator_joint_at_limit_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimator_disturbance_observable." + axes[i],
                &estimator_disturbance_observable_[i]));
        }
        for (int i=0;i<4;++i) for (int j=0;j<4;++j)
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name,
                "estimated_disturbance.covariance."+std::to_string(i)+"."+std::to_string(j),
                &disturbance_covariance_[4*i+j]));
        for (std::size_t i = 0; i < estimator_model_readback_.size(); ++i)
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimator_nominal_parameter." + std::to_string(i),
                &estimator_model_readback_[i]));
        // 0-3: payload
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[0].name, &payload_mass));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[1].name, &payload_Ixx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[2].name, &payload_Iyy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[3].name, &payload_Izz));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[4].name, &gravity_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[5].name, &control_power_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[6].name, &control_energy_));
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
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        reset_joint_simulation_state();
        commands_held_ = false;
        RCLCPP_INFO(
            rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "Activating... please wait...");
        for (std::size_t i = 0; i < hw_joint_struct_.size(); ++i)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                "[%s] default sim joint %zu position %.6f",
                robot_prefix.c_str(), i, hw_joint_struct_[i].current_state_.filtered_position);
        }
        RCLCPP_INFO(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"), "System successfully activated!");
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
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        delta_seconds = period.seconds();
        time_seconds = time.seconds();
        apply_pending_reset_request();

        DM q = DM::vertcat({hw_joint_struct_[0].current_state_.position,
                            hw_joint_struct_[1].current_state_.position,
                            hw_joint_struct_[2].current_state_.position,
                            hw_joint_struct_[3].current_state_.position});

        DM base_T = DM::vertcat({0.190, 0.000, -0.120, 3.141592653589793, 0.000, 0.000});
        DM world_T = DM::vertcat({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        DM tipOffset = DM::vertcat({0.0, 0.0, 0.04, 0.0, 0.0, 0.0});

        std::vector<DM> fk_args = {q, base_T, world_T, tipOffset};
        T_i_ = utils_service.forward_kinematics(fk_args);

        DM c_sample = DM::vertcat({5e-12, -1e-12, 16e-12,
                                   73.563e-12, -0.091e-12, -0.734e-12,
                                   17e-12, -26e-12, 2e-12,
                                   -0.030e-12, -12e-12, -98e-12});
        DM r_com_body = DM::vertcat({0.0, 0.0, 0.0});

        std::vector<DM> fkcom_args = {q, c_sample, r_com_body, base_T, world_T, tipOffset};
        T_com_i_ = utils_service.forward_kinematics_com(fkcom_args);
        // Time step for this cycle
        casadi::DM dt_dm(delta_seconds);

        // The gripper has a separate scalar model and retains its existing filter.
        for (std::size_t i = 4; i < get_hardware_info().joints.size(); ++i)
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
            const double filtered_position = dense_vector_value(x_est_list_[i], 0);
            const double filtered_velocity = dense_vector_value(x_est_list_[i], 1);
            const double estimated_acceleration = dense_vector_value(x_est_list_[i], 2);
            hw_joint_struct_[i].current_state_.filtered_position = filtered_position;
            hw_joint_struct_[i].current_state_.filtered_velocity = filtered_velocity;
            hw_joint_struct_[i].current_state_.estimated_acceleration = estimated_acceleration;
            hw_joint_struct_[i].current_state_.acceleration = estimated_acceleration;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimReachSystemMultiInterfaceHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        apply_pending_reset_request();
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        if (commands_held_)
        {
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                *node_frames_interface_->get_clock(),
                2000,
                "[%s] manipulator commands currently held after reset",
                robot_prefix.c_str());
            for (auto &joint : hw_joint_struct_)
            {
                joint.command_state_.velocity = 0.0;
                joint.command_state_.acceleration = 0.0;
                joint.command_state_.current = 0.0;
                joint.command_state_.effort = 0.0;
                joint.command_state_.computed_effort = 0.0;
                joint.current_state_.sim_time = time_seconds;
                joint.current_state_.sim_period = delta_seconds;
            }
            control_power_ = 0.0;
        }

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
            double I_cmd = 0.0;
            if (std::abs(tau_cmd) > kTorqueCurrentZeroEpsilon)
            {
                const bool pos_tau = (tau_cmd > 0.0);
                I_cmd = arm_actuator_model::torque_to_current(
                    hw_joint_struct_[j].actuator_Properties_.kt,
                    pos_tau ? hw_joint_struct_[j].actuator_Properties_.forward_I_static
                            : hw_joint_struct_[j].actuator_Properties_.backward_I_static,
                    tau_cmd);
            }

            // clamp in current using your existing hard limiter that also enforces position limits
            const double I_safe = hw_joint_struct_[j].enforce_hard_limits(I_cmd);

            // current -> torque
            double tau_safe = 0.0;
            if (std::abs(I_safe) > kTorqueCurrentZeroEpsilon)
            {
                const bool pos_cur = (I_safe > 0.0);
                tau_safe = arm_actuator_model::current_to_torque(
                    hw_joint_struct_[j].actuator_Properties_.kt,
                    pos_cur ? hw_joint_struct_[j].actuator_Properties_.forward_I_static
                            : hw_joint_struct_[j].actuator_Properties_.backward_I_static,
                    I_safe);
            }

            hw_joint_struct_[j].current_state_.current = I_safe;
            hw_joint_struct_[j].current_state_.effort = tau_safe;
            hw_joint_struct_[j].current_state_.computed_effort = tau_cmd;
            hw_joint_struct_[j].current_state_.computed_effort_uncertainty = tau_cmd - tau_safe;
            arm_torques.push_back(tau_safe);
        };

        apply_motor_lock();

        control_power_ = 0.0;

        for (int j = 0; j < 5; ++j)
        {
            const double tau = arm_torques[j].scalar();
            const double qdot = hw_joint_struct_[j].current_state_.velocity;
            control_power_ += std::abs(tau * qdot);
        }

        control_energy_ += control_power_ * delta_seconds;

        // number of dynamic joints in the model
        constexpr std::size_t n_dyn = 4;

        // build a single column mask of length 4
        casadi::DM lock_mask = casadi::DM::zeros(static_cast<int>(n_dyn), 1);
        for (std::size_t j = 0; j < n_dyn; ++j)
        {
            lock_mask(j) = is_locked_[j] ? 1.0 : 0.0;
        }

        // Injection is plant-only; actuator input to the EKF remains unchanged.
        const auto injection = disturbance_receiver_ ? disturbance_receiver_->sample() : msg::DisturbanceCommand{};
        auto plant_torque = DM::vertcat(arm_torques);
        for (int j=0; j<4; ++j) plant_torque(j) += injection.joint_torques[j];
        // call dynamics with the mask
        arm_simulate_argument = {arm_state, plant_torque, delta_seconds, manipulator_parameters_, DM(endeffector_mass_),
                                 DM(endeffector_damping_), DM(endeffector_stiffness_), lock_mask, DM(baumgarte_alpha_)};
        try {
            arm_sim = arm_time_integration::plant(utils_service.manipulator_dynamics, arm_simulate_argument);
        } catch (const std::exception &error) {
            estimator_valid_ = 0.0;
            RCLCPP_ERROR(rclcpp::get_logger("SimReachSystemMultiInterfaceHardware"),
                         "Arm plant integration failed (dt=%.9g s): %s", delta_seconds, error.what());
            return hardware_interface::return_type::ERROR;
        }
        arm_next_states = dense_vector(arm_sim.at(0), 10);
        if (disturbance_receiver_) disturbance_receiver_->completed(injection, delta_seconds);
        for (int j = 0; j < 4; ++j)
            estimator_applied_input_(j) = arm_torques[j];
        estimator_completed_lock_mask_ = lock_mask;
        estimator_completed_lock_alpha_ = baumgarte_alpha_;
        estimator_completed_dt_ = delta_seconds;

        // clamp the NEW state, then write it back
        for (int j = 0; j < 5; ++j)
        {
            double q = arm_next_states[j];
            double qd = arm_next_states[5 + j];

            const auto &lim = hw_joint_struct_[j].limits_;

            if (j < 4)
                estimator_completed_contact_(j) =
                    (q <= lim.position_min + 1e-9 || q >= lim.position_max - 1e-9) ? 1.0 : 0.0;
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
        sim_step_ += 1.0;
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

                try
                {
                    const auto v_dense = dense_vector(T_i_[i], 7); // [x y z qw qx qy qz]
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
                catch (const std::exception &)
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

                try
                {
                    const auto v_dense = dense_vector(T_com_i_[i], 7); // [x y z qw qx qy qz]
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
                catch (const std::exception &)
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
