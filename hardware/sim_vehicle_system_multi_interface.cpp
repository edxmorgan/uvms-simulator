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

#include "ros2_control_blue_reach_5/sim_vehicle_system_multi_interface.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>
#if __has_include("ros2_control_blue_reach_5/dynamics_params.hpp")
//   #pragma message("Private parameters enabled (header found)")
#include "ros2_control_blue_reach_5/dynamics_params.hpp"
#else
//   #pragma message("Fallback parameters used")
#endif

using namespace casadi;

namespace ros2_control_blue_reach_5
{
    namespace
    {
        std::array<double, 48> default_thrust_configuration_matrix()
        {
            return {
                0.707, 0.707, -0.707, -0.707, 0.0, 0.0, 0.0, 0.0,
                -0.707, 0.707, -0.707, 0.707, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0,
                0.06, -0.06, 0.06, -0.06, -0.218, -0.218, 0.218, 0.218,
                0.06, 0.06, -0.06, -0.06, 0.12, -0.12, 0.12, -0.12,
                -0.1888, 0.1888, 0.1888, -0.1888, 0.0, 0.0, 0.0, 0.0};
        }

        bool valid_thrust_configuration_matrix(const std::array<double, 48> &matrix)
        {
            return std::any_of(matrix.begin(), matrix.end(), [](const double value)
                               { return std::abs(value) > 1.0e-12; });
        }

        DM thrust_configuration_dm(const std::array<double, 48> &matrix)
        {
            DM result = DM::zeros(6, 8);
            for (std::size_t row = 0; row < 6; ++row)
            {
                for (std::size_t col = 0; col < 8; ++col)
                {
                    result(row, col) = matrix[row * 8 + col];
                }
            }
            return result;
        }

        double camera_mount_pwm_to_pitch(const double pwm)
        {
            constexpr double kMinPwm = 1100.0;
            constexpr double kNeutralPwm = 1500.0;
            constexpr double kMaxPwm = 1900.0;
            constexpr double kMaxPitchRad = 0.7853981633974483;
            const double clamped_pwm = std::clamp(pwm, kMinPwm, kMaxPwm);
            if (clamped_pwm >= kNeutralPwm)
            {
                return (clamped_pwm - kNeutralPwm) / (kMaxPwm - kNeutralPwm) * kMaxPitchRad;
            }
            return (clamped_pwm - kNeutralPwm) / (kNeutralPwm - kMinPwm) * kMaxPitchRad;
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

        std::vector<double> default_vehicle_simulation_parameters()
        {
            return {
                3.72028553e+01, 2.21828075e+01, 6.61734807e+01, 3.38909801e+00,
                6.41362046e-01, 6.41362034e-01, 3.38909800e+00, 1.39646394e+00,
                4.98032205e-01, 2.53118738e+00, 1.05000000e+02, 9.78296453e+01,
                8.27479545e-01, 1.36822559e-01, 4.25841171e+00, -7.36416666e+01,
                -3.36082112e+01, -8.94055107e+01, -2.98736214e+00, -1.57921531e+00,
                -3.39766499e+00, -1.47912104e-04, -5.16373030e-04, -9.85522538e+01,
                -3.05907788e-02, -1.27877517e-01, -1.63514832e+00,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        ros2_control_blue_reach_5::msg::SimVehicleDynamics default_vehicle_dynamics()
        {
            auto msg = ros2_control_blue_reach_5::msg::SimVehicleDynamics();
            const auto values = default_vehicle_simulation_parameters();
            msg.m_x_du = values[0];
            msg.m_y_dv = values[1];
            msg.m_z_dw = values[2];
            msg.mz_g_x_dq = values[3];
            msg.mz_g_y_dp = values[4];
            msg.mz_g_k_dv = values[5];
            msg.mz_g_m_du = values[6];
            msg.i_x_k_dp = values[7];
            msg.i_y_m_dq = values[8];
            msg.i_z_n_dr = values[9];
            msg.weight = values[10];
            msg.buoyancy = values[11];
            msg.x_g_weight_minus_x_b_buoyancy = values[12];
            msg.y_g_weight_minus_y_b_buoyancy = values[13];
            msg.z_g_weight_minus_z_b_buoyancy = values[14];
            msg.x_u = values[15];
            msg.y_v = values[16];
            msg.z_w = values[17];
            msg.k_p = values[18];
            msg.m_q = values[19];
            msg.n_r = values[20];
            msg.x_uu = values[21];
            msg.y_vv = values[22];
            msg.z_ww = values[23];
            msg.k_pp = values[24];
            msg.m_qq = values[25];
            msg.n_rr = values[26];
            for (std::size_t i = 0; i < msg.current_velocity.size(); ++i)
            {
                msg.current_velocity[i] = values[27 + i];
            }
            msg.thrust_configuration_matrix = default_thrust_configuration_matrix();
            return msg;
        }

        std::vector<double> pack_vehicle_dynamics(
            const ros2_control_blue_reach_5::msg::SimVehicleDynamics &msg)
        {
            return {
                msg.m_x_du,
                msg.m_y_dv,
                msg.m_z_dw,
                msg.mz_g_x_dq,
                msg.mz_g_y_dp,
                msg.mz_g_k_dv,
                msg.mz_g_m_du,
                msg.i_x_k_dp,
                msg.i_y_m_dq,
                msg.i_z_n_dr,
                msg.weight,
                msg.buoyancy,
                msg.x_g_weight_minus_x_b_buoyancy,
                msg.y_g_weight_minus_y_b_buoyancy,
                msg.z_g_weight_minus_z_b_buoyancy,
                msg.x_u,
                msg.y_v,
                msg.z_w,
                msg.k_p,
                msg.m_q,
                msg.n_r,
                msg.x_uu,
                msg.y_vv,
                msg.z_ww,
                msg.k_pp,
                msg.m_qq,
                msg.n_rr,
                msg.current_velocity[0],
                msg.current_velocity[1],
                msg.current_velocity[2],
                msg.current_velocity[3],
                msg.current_velocity[4],
                msg.current_velocity[5]};
        }

        std::vector<casadi::DM> to_dm_vector(const std::vector<double> &values)
        {
            std::vector<casadi::DM> result;
            result.reserve(values.size());
            for (const double value : values)
            {
                result.emplace_back(value);
            }
            return result;
        }
    } // namespace

    SimVehicleSystemMultiInterfaceHardware::~SimVehicleSystemMultiInterfaceHardware()
    {
        stop_ros_interfaces();
    }

    void SimVehicleSystemMultiInterfaceHardware::reset_vehicle_estimators()
    {
        x_est_ = casadi::DM::zeros(18, 1);
        P_est_ = casadi::DM::eye(18) * 0.001;
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
        }
    }

    void SimVehicleSystemMultiInterfaceHardware::reset_vehicle_simulation_state()
    {
        hw_vehicle_struct.current_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.async_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.command_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.estimate_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.depth_from_pressure2 = hw_vehicle_struct.default_state_.position_z;
        hw_vehicle_struct.dvl_state = {};
        hw_vehicle_struct.imu_state = {};
        hw_vehicle_struct.sim_time = 0.0;
        hw_vehicle_struct.sim_period = 0.0;
        hw_vehicle_struct.camera_mountPitch_pwm = 1500.0;
        updateCameraMountPitchState();
        control_power_ = 0.0;
        control_energy_ = 0.0;
        delta_seconds = 0.0;
        time_seconds = 0.0;

        for (auto &thruster : hw_vehicle_struct.hw_thrust_structs_)
        {
            thruster.current_state_ = thruster.default_state_;
            thruster.async_state_ = thruster.default_state_;
            thruster.command_state_ = thruster.default_state_;
            thruster.command_state_.command_pwm = thruster.neutral_pwm;
            thruster.current_state_.rc_pwm = thruster.neutral_pwm;
        }

        {
            std::lock_guard<std::mutex> wrench_lock(contact_wrench_mutex_);
            contact_wrench_body_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        }

        commands_held_ = true;
        reset_vehicle_estimators();
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "[%s] reset simulated vehicle state for %zu thrusters; commands held until release",
            hw_vehicle_struct.robot_prefix.c_str(),
            hw_vehicle_struct.hw_thrust_structs_.size());
    }

    void SimVehicleSystemMultiInterfaceHardware::reset_vehicle_simulation_state(
        const ros2_control_blue_reach_5::srv::ResetSimUvms::Request &request)
    {
        reset_vehicle_simulation_state();

        if (request.use_vehicle_state)
        {
            auto requested_state = hw_vehicle_struct.default_state_;
            requested_state.position_x = request.vehicle_pose[0];
            requested_state.position_y = request.vehicle_pose[1];
            requested_state.position_z = request.vehicle_pose[2];
            requested_state.setEuler(
                request.vehicle_pose[3],
                request.vehicle_pose[4],
                request.vehicle_pose[5]);
            requested_state.u = request.vehicle_twist[0];
            requested_state.v = request.vehicle_twist[1];
            requested_state.w = request.vehicle_twist[2];
            requested_state.p = request.vehicle_twist[3];
            requested_state.q = request.vehicle_twist[4];
            requested_state.r = request.vehicle_twist[5];
            requested_state.Fx = request.vehicle_wrench[0];
            requested_state.Fy = request.vehicle_wrench[1];
            requested_state.Fz = request.vehicle_wrench[2];
            requested_state.Tx = request.vehicle_wrench[3];
            requested_state.Ty = request.vehicle_wrench[4];
            requested_state.Tz = request.vehicle_wrench[5];

            hw_vehicle_struct.current_state_ = requested_state;
            hw_vehicle_struct.async_state_ = requested_state;
            hw_vehicle_struct.estimate_state_ = requested_state;
            hw_vehicle_struct.command_state_ = requested_state;
            hw_vehicle_struct.command_state_.Fx = 0.0;
            hw_vehicle_struct.command_state_.Fy = 0.0;
            hw_vehicle_struct.command_state_.Fz = 0.0;
            hw_vehicle_struct.command_state_.Tx = 0.0;
            hw_vehicle_struct.command_state_.Ty = 0.0;
            hw_vehicle_struct.command_state_.Tz = 0.0;
            hw_vehicle_struct.depth_from_pressure2 = requested_state.position_z;
        }

        commands_held_ = request.hold_commands;
        if (request.set_vehicle_dynamics)
        {
            use_coupled_dynamics_ = request.use_coupled_dynamics;
            vehicle_parameters_new = to_dm_vector(pack_vehicle_dynamics(request.vehicle_dynamics));
            thrust_configuration_matrix_ = request.vehicle_dynamics.thrust_configuration_matrix;
        }
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "[%s] reset simulated vehicle with requested state; hold=%s pose=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
            hw_vehicle_struct.robot_prefix.c_str(),
            commands_held_ ? "true" : "false",
            hw_vehicle_struct.current_state_.position_x,
            hw_vehicle_struct.current_state_.position_y,
            hw_vehicle_struct.current_state_.position_z,
            hw_vehicle_struct.current_state_.roll,
            hw_vehicle_struct.current_state_.pitch,
            hw_vehicle_struct.current_state_.yaw);
    }

    void SimVehicleSystemMultiInterfaceHardware::stop_ros_interfaces() noexcept
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
        static_tf_broadcaster_.reset();
        executor_.reset();
        node_topics_interface_.reset();
    }

    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_init(
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
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");
        utils_service.vehicle_dynamics = utils_service.load_casadi_fun("Vnext_reg", "libUV_xnext.so");
        utils_service.genForces2propThrust = utils_service.load_casadi_fun("F_thrusters", "libF_thrust.so");
        utils_service.thrust2rads = utils_service.load_casadi_fun("thrusts_to_rads", "libTHRUST_RAD.so");
        utils_service.uv_Exkalman_update = utils_service.load_casadi_fun("ekf_update", "libEKF_next.so");
        // utils_service.uv_dynamic_Exkalman_update = utils_service.load_casadi_fun("mb_ekf_update", "libmodelbasedEKF_next.so");
        utils_service.pwm_to_thrusts = utils_service.load_casadi_fun("pwm_to_thrusts", "libPWM_THRUST.so");
        utils_service.thruster_f2body_f = utils_service.load_casadi_fun("F_thruster_body", "libF_thruster_body.so");

        hw_vehicle_struct.world_frame_id = get_hardware_info().hardware_parameters.at("world_frame_id");
        hw_vehicle_struct.body_frame_id = get_hardware_info().hardware_parameters.at("body_frame_id");
        hw_vehicle_struct.map_frame_id = get_hardware_info().hardware_parameters.at("map_frame_id");
        hw_vehicle_struct.robot_prefix = get_hardware_info().hardware_parameters.at("prefix");

        if (get_hardware_info().hardware_parameters.find("world_frame_id") == get_hardware_info().hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'world_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.world_frame_id = get_hardware_info().hardware_parameters.at("world_frame_id");

        if (get_hardware_info().hardware_parameters.find("body_frame_id") == get_hardware_info().hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'body_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.body_frame_id = get_hardware_info().hardware_parameters.at("body_frame_id");

        if (get_hardware_info().hardware_parameters.find("map_frame_id") == get_hardware_info().hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'map_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.map_frame_id = get_hardware_info().hardware_parameters.at("map_frame_id");

        if (get_hardware_info().hardware_parameters.find("prefix") == get_hardware_info().hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'prefix' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.robot_prefix = get_hardware_info().hardware_parameters.at("prefix");

        if (get_hardware_info().hardware_parameters.find("use_pwm") == get_hardware_info().hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'use_pwm' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        const std::string use_pwm_str = get_hardware_info().hardware_parameters.at("use_pwm"); // e.g., "true", "false", "1", "0"
        hw_vehicle_struct.use_pwm =
            use_pwm_str == "true" || use_pwm_str == "True" || use_pwm_str == "1";
        const auto default_dynamics = default_vehicle_dynamics();
        vehicle_parameters_new = to_dm_vector(pack_vehicle_dynamics(default_dynamics));
        thrust_configuration_matrix_ = default_dynamics.thrust_configuration_matrix;

        bool same_initial_conditions = false;
        if (get_hardware_info().hardware_parameters.find("same_initial_conditions") != get_hardware_info().hardware_parameters.cend())
        {
            const std::string same_ic_str = get_hardware_info().hardware_parameters.at("same_initial_conditions");
            same_initial_conditions =
                same_ic_str == "true" || same_ic_str == "True" || same_ic_str == "1";
        }

        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "[%s] frames: world=%s body=%s map=%s",
            hw_vehicle_struct.robot_prefix.c_str(),
            hw_vehicle_struct.world_frame_id.c_str(),
            hw_vehicle_struct.body_frame_id.c_str(),
            hw_vehicle_struct.map_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "use_pwm: %s", use_pwm_str.c_str());

        if (same_initial_conditions)
        {
            map_position_x = 0.0;
            map_position_y = 0.0;
            map_position_z = 0.0;
            map_orientation_yaw = 0.0;
        }
        else
        {
            // Use the robot_prefix as a seed
            std::size_t seed_val = std::hash<std::string>{}(hw_vehicle_struct.robot_prefix);
            std::mt19937 gen(seed_val + 23);

            std::uniform_real_distribution<> dis_x(-5.0, 5.0);
            std::uniform_real_distribution<> dis_y(-5.0, 5.0);
            std::uniform_real_distribution<> dis_z(0.0, 0.0);
            std::uniform_real_distribution<double> dis_yaw(-M_PI, M_PI);

            map_position_x = dis_x(gen);
            map_position_y = dis_y(gen);
            map_position_z = dis_z(gen);
            map_orientation_yaw = dis_yaw(gen);
        }

        // roll = 0, pitch = 0
        double half = 0.5 * map_orientation_yaw;
        map_orientation_w = std::cos(half);
        map_orientation_x = 0.0;
        map_orientation_y = 0.0;
        map_orientation_z = std::sin(half);

        blue::dynamics::Vehicle::Pose_vel initial_state{
            0.0, 0.0, 0.0,      // position: x, y, z
            0.0, 0.0, 0.0,      // Orientation: r, p, y
            1.0, 0.0, 0.0, 0.0, // Orientation: qw, qx, qy, qz
            0.0, 0.0, 0.0,      // Linear velocities: vx, vy, vz
            0.0, 0.0, 0.0,      // Angular velocities: wx, wy, wz
            0.0, 0.0, 0.0,      // Forces: Fx, Fy, Fz
            0.0, 0.0, 0.0       // Torques: Tx, Ty, Tz

        };

        hw_vehicle_struct.set_vehicle_name("blue ROV heavy 0", initial_state);

        hw_vehicle_struct.thrustSizeAllocation(get_hardware_info().joints.size());

        for (const hardware_interface::ComponentInfo &joint : get_hardware_info().joints)
        {
            Thruster::State defaultState{};
            hw_vehicle_struct.hw_thrust_structs_.emplace_back(joint.name, defaultState);
            // SimVehicleSystemMultiInterfaceHardware has exactly 6 joint state interfaces
            if (joint.state_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu state interfaces. 6 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu command interfaces. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };
        };

        for (const hardware_interface::ComponentInfo &gpio : get_hardware_info().gpios)
        {
            // SimVehicleSystemMultiInterfaceHardware has exactly 83 gpio state interfaces
            if (gpio.state_interfaces.size() != 83)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 83 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // SimVehicleSystemMultiInterfaceHardware has exactly 28 gpio command interfaces
            if (gpio.command_interfaces.size() != 28)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu command interfaces. 28 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
        // declare and get parameters needed for controller operations
        // setup realtime buffers, ROS publishers ...
        try
        {
            // Initialize node
            node_topics_interface_ = std::make_shared<rclcpp::Node>(system_name + "_topics_interface");

            // Initialize executor and add node
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor_->add_node(node_topics_interface_);

            // Start spinning in a separate thread
            spin_thread_ = std::thread([this]()
                                       { executor_->spin(); });

            RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                        "Started executor and spinning node_topics_interface: %s", node_topics_interface_->get_name());

            // Initialize the StaticTransformBroadcaster
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_topics_interface_);

            // tf publisher
            transform_publisher_ = rclcpp::create_publisher<tf>(node_topics_interface_,
                                                                DEFAULT_TRANSFORM_TOPIC,
                                                                rclcpp::SystemDefaultsQoS());
            realtime_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(
                    transform_publisher_);

            auto &transform_message = realtime_transform_publisher_->msg_;
            transform_message.transforms.resize(2);

            contact_wrench_sub_ =
                node_topics_interface_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                    "contact_wrench_body",
                    rclcpp::QoS(10),
                    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
                    {
                        // Topic contract: WrenchStamped is already expressed in
                        // the vehicle body frame and already uses the dynamics
                        // sign convention [Fx, Fy, Fz, Tx, Ty, Tz].
                        std::lock_guard<std::mutex> lock(contact_wrench_mutex_);
                        contact_wrench_body_[0] = msg->wrench.force.x;
                        contact_wrench_body_[1] = msg->wrench.force.y;
                        contact_wrench_body_[2] = msg->wrench.force.z;
                        contact_wrench_body_[3] = msg->wrench.torque.x;
                        contact_wrench_body_[4] = msg->wrench.torque.y;
                        contact_wrench_body_[5] = msg->wrench.torque.z;
                    });

            auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
            camera_mount_pitch_subscriber_ =
                node_topics_interface_->create_subscription<std_msgs::msg::Float32>(
                    "/alpha/cameraMountPitch",
                    volatile_qos,
                    std::bind(&SimVehicleSystemMultiInterfaceHardware::cameraMountPitch_callback, this, std::placeholders::_1));
        }
        catch (const std::exception &e)
        {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
                e.what());
            return CallbackReturn::ERROR;
        }

        reset_vehicle_estimators();

        // Process noise covariance: 18x18, scaled by 0.01
        casadi::DM Q_vector = casadi::DM::zeros(18, 1);
        Q_vector(0) = 0.001;
        Q_vector(1) = 0.001;
        Q_vector(2) = 0.001;
        Q_vector(3) = 0.001;
        Q_vector(4) = 0.001;
        Q_vector(5) = 0.001;
        Q_vector(6) = 0.001;
        Q_vector(7) = 0.001;
        Q_vector(8) = 0.001;
        Q_vector(9) = 0.001;
        Q_vector(10) = 0.001;
        Q_vector(11) = 0.001;
        Q_ = casadi::DM::diag(Q_vector);

        // Measurement noise R_
        casadi::DM R_vector = casadi::DM::zeros(7, 1); // 7x1 vector
        R_vector(0) = 0.01;                            // z_pressure noise variance
        R_vector(1) = 0.005;                           // IMU roll noise variance
        R_vector(2) = 0.005;                           // IMU pitch noise variance
        R_vector(3) = 0.005;                           // IMU yaw noise variance
        R_vector(4) = 0.005;                           // DVL vx noise variance
        R_vector(5) = 0.005;                           // DVL vy noise variance
        R_vector(6) = 0.005;                           // DVL vz noise variance
        R_ = casadi::DM::diag(R_vector);

        auto reset_state_callback =
            [this](
                const std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Request> request,
                std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Response> response)
        {
            std::lock_guard<std::mutex> lock(simulation_state_mutex_);
            if (request->set_vehicle_dynamics &&
                !valid_thrust_configuration_matrix(request->vehicle_dynamics.thrust_configuration_matrix))
            {
                response->success = false;
                response->message = "vehicle_dynamics.thrust_configuration_matrix must contain a nonzero 6x8 row-major matrix";
                return;
            }
            reset_vehicle_simulation_state(*request);
            response->success = true;
            response->message = "reset simulated vehicle to requested state";
        };

        reset_service_ = node_topics_interface_->create_service<ros2_control_blue_reach_5::srv::ResetSimUvms>(
            "/" + hw_vehicle_struct.robot_prefix + "reset_sim_vehicle",
            reset_state_callback);

        release_service_ = node_topics_interface_->create_service<std_srvs::srv::Trigger>(
            "/" + hw_vehicle_struct.robot_prefix + "release_sim_vehicle",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                std::lock_guard<std::mutex> lock(simulation_state_mutex_);
                commands_held_ = false;
                RCLCPP_INFO(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "[%s] released simulated vehicle commands",
                    hw_vehicle_struct.robot_prefix.c_str());
                response->success = true;
                response->message = "released simulated vehicle commands";
            });

        dynamics_service_ = node_topics_interface_->create_service<ros2_control_blue_reach_5::srv::SetSimDynamics>(
            "/" + hw_vehicle_struct.robot_prefix + "set_sim_vehicle_dynamics",
            [this](
                const std::shared_ptr<ros2_control_blue_reach_5::srv::SetSimDynamics::Request> request,
                std::shared_ptr<ros2_control_blue_reach_5::srv::SetSimDynamics::Response> response)
            {
                if (!request->set_vehicle_dynamics)
                {
                    response->success = false;
                    response->message = "set_vehicle_dynamics must be true";
                    return;
                }
                if (!valid_thrust_configuration_matrix(request->vehicle.thrust_configuration_matrix))
                {
                    response->success = false;
                    response->message = "vehicle.thrust_configuration_matrix must contain a nonzero 6x8 row-major matrix";
                    return;
                }
                std::lock_guard<std::mutex> lock(simulation_state_mutex_);
                use_coupled_dynamics_ = request->use_coupled_dynamics;
                vehicle_parameters_new = to_dm_vector(pack_vehicle_dynamics(request->vehicle));
                thrust_configuration_matrix_ = request->vehicle.thrust_configuration_matrix;
                response->success = true;
                response->message = "updated simulated vehicle dynamics parameters";
                RCLCPP_INFO(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "[%s] updated simulated vehicle dynamics parameter vector (%zu values), use_coupled_dynamics=%s",
                    hw_vehicle_struct.robot_prefix.c_str(),
                    vehicle_parameters_new.size(),
                    use_coupled_dynamics_ ? "true" : "false");
            });

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Initialized P_est_, Q_, and R_ for Kalman filter.");
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "configure successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_cleanup(const rclcpp_lifecycle::State &)
    {
        stop_ros_interfaces();
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "Shutting down the simulated vehicle system interface.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    SimVehicleSystemMultiInterfaceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.acceleration));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.rc_pwm));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period));
        }

        // 0-2: Position
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[0].name, &hw_vehicle_struct.current_state_.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[1].name, &hw_vehicle_struct.current_state_.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[2].name, &hw_vehicle_struct.current_state_.position_z));

        // 3-5: Orientation (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[3].name, &hw_vehicle_struct.current_state_.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[4].name, &hw_vehicle_struct.current_state_.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[5].name, &hw_vehicle_struct.current_state_.yaw));

        // 6-9: Body Orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[6].name, &hw_vehicle_struct.current_state_.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[7].name, &hw_vehicle_struct.current_state_.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[8].name, &hw_vehicle_struct.current_state_.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[9].name, &hw_vehicle_struct.current_state_.orientation_z));

        // 10-12: Linear Velocity (u, v, w)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[10].name, &hw_vehicle_struct.current_state_.u));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[11].name, &hw_vehicle_struct.current_state_.v));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[12].name, &hw_vehicle_struct.current_state_.w));

        // 13-15: Angular Velocity (p, q, r)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[13].name, &hw_vehicle_struct.current_state_.p));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[14].name, &hw_vehicle_struct.current_state_.q));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[15].name, &hw_vehicle_struct.current_state_.r));

        // 16-18: Linear Acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[16].name, &hw_vehicle_struct.current_state_.du));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[17].name, &hw_vehicle_struct.current_state_.dv));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[18].name, &hw_vehicle_struct.current_state_.dw));

        // 19-21: Angular Acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[19].name, &hw_vehicle_struct.current_state_.dp));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[20].name, &hw_vehicle_struct.current_state_.dq));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[21].name, &hw_vehicle_struct.current_state_.dr));

        // 22-24: Force
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[22].name, &hw_vehicle_struct.current_state_.Fx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[23].name, &hw_vehicle_struct.current_state_.Fy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[24].name, &hw_vehicle_struct.current_state_.Fz));

        // 25-27: Torque
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[25].name, &hw_vehicle_struct.current_state_.Tx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[26].name, &hw_vehicle_struct.current_state_.Ty));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[27].name, &hw_vehicle_struct.current_state_.Tz));

        // 28-29: Simulation time and period
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[28].name, &hw_vehicle_struct.sim_time));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[29].name, &hw_vehicle_struct.sim_period));

        // 30-42: IMU interfaces
        // 30-32: IMU angles (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[30].name, &hw_vehicle_struct.imu_state.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[31].name, &hw_vehicle_struct.imu_state.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[32].name, &hw_vehicle_struct.imu_state.yaw));

        // 33-35: IMU angles (roll, pitch, yaw) unwrap
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[33].name, &hw_vehicle_struct.imu_state.roll_unwrap));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[34].name, &hw_vehicle_struct.imu_state.pitch_unwrap));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[35].name, &hw_vehicle_struct.imu_state.yaw_unwrap));

        // 36-39: IMU orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[36].name, &hw_vehicle_struct.imu_state.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[37].name, &hw_vehicle_struct.imu_state.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[38].name, &hw_vehicle_struct.imu_state.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[39].name, &hw_vehicle_struct.imu_state.orientation_z));

        // 40-42: IMU angular velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[40].name, &hw_vehicle_struct.imu_state.angular_vel_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[41].name, &hw_vehicle_struct.imu_state.angular_vel_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[42].name, &hw_vehicle_struct.imu_state.angular_vel_z));

        // 43-45: IMU linear acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[43].name, &hw_vehicle_struct.imu_state.linear_acceleration_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[44].name, &hw_vehicle_struct.imu_state.linear_acceleration_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[45].name, &hw_vehicle_struct.imu_state.linear_acceleration_z));

        // 46: Depth measurement
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[46].name, &hw_vehicle_struct.depth_from_pressure2));

        // 47-49: DVL gyro (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[47].name, &hw_vehicle_struct.dvl_state.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[48].name, &hw_vehicle_struct.dvl_state.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[49].name, &hw_vehicle_struct.dvl_state.yaw));

        // 50-52: DVL speed (x, y, z)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[50].name, &hw_vehicle_struct.dvl_state.vx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[51].name, &hw_vehicle_struct.dvl_state.vy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[52].name, &hw_vehicle_struct.dvl_state.vz));

        // 53-55: State Estimation Position
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[53].name, &hw_vehicle_struct.estimate_state_.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[54].name, &hw_vehicle_struct.estimate_state_.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[55].name, &hw_vehicle_struct.estimate_state_.position_z));

        // 56-58: State Estimation Orientation (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[56].name, &hw_vehicle_struct.estimate_state_.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[57].name, &hw_vehicle_struct.estimate_state_.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[58].name, &hw_vehicle_struct.estimate_state_.yaw));

        // 59-62: State Estimation Body Orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[59].name, &hw_vehicle_struct.estimate_state_.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[60].name, &hw_vehicle_struct.estimate_state_.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[61].name, &hw_vehicle_struct.estimate_state_.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[62].name, &hw_vehicle_struct.estimate_state_.orientation_z));

        // 63-65: State Estimation Linear Velocity (u, v, w)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[63].name, &hw_vehicle_struct.estimate_state_.u));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[64].name, &hw_vehicle_struct.estimate_state_.v));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[65].name, &hw_vehicle_struct.estimate_state_.w));

        // 66-68: State Estimation Angular Velocity (p, q, r)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[66].name, &hw_vehicle_struct.estimate_state_.p));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[67].name, &hw_vehicle_struct.estimate_state_.q));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[68].name, &hw_vehicle_struct.estimate_state_.r));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[69].name, &P_diag_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[70].name, &P_diag_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[71].name, &P_diag_[2]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[72].name, &P_diag_[3]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[73].name, &P_diag_[4]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[74].name, &P_diag_[5]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[75].name, &P_diag_[6]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[76].name, &P_diag_[7]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[77].name, &P_diag_[8]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[78].name, &P_diag_[9]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[79].name, &P_diag_[10]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[80].name, &P_diag_[11]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[81].name, &control_power_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].state_interfaces[82].name, &control_energy_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SimVehicleSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[0].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[1].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[2].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[3].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[4].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[5].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[6].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().joints[7].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[0].name, &hw_vehicle_struct.command_state_.position_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[1].name, &hw_vehicle_struct.command_state_.position_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[2].name, &hw_vehicle_struct.command_state_.position_z));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[3].name, &hw_vehicle_struct.command_state_.roll));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[4].name, &hw_vehicle_struct.command_state_.pitch));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[5].name, &hw_vehicle_struct.command_state_.yaw));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[6].name, &hw_vehicle_struct.command_state_.orientation_w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[7].name, &hw_vehicle_struct.command_state_.orientation_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[8].name, &hw_vehicle_struct.command_state_.orientation_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[9].name, &hw_vehicle_struct.command_state_.orientation_z));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[10].name, &hw_vehicle_struct.command_state_.u));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[11].name, &hw_vehicle_struct.command_state_.v));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[12].name, &hw_vehicle_struct.command_state_.w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[13].name, &hw_vehicle_struct.command_state_.p));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[14].name, &hw_vehicle_struct.command_state_.q));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[15].name, &hw_vehicle_struct.command_state_.r));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[16].name, &hw_vehicle_struct.command_state_.du));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[17].name, &hw_vehicle_struct.command_state_.dv));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[18].name, &hw_vehicle_struct.command_state_.dw));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[19].name, &hw_vehicle_struct.command_state_.dp));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[20].name, &hw_vehicle_struct.command_state_.dq));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[21].name, &hw_vehicle_struct.command_state_.dr));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[22].name, &hw_vehicle_struct.command_state_.Fx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[23].name, &hw_vehicle_struct.command_state_.Fy));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[24].name, &hw_vehicle_struct.command_state_.Fz));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[25].name, &hw_vehicle_struct.command_state_.Tx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[26].name, &hw_vehicle_struct.command_state_.Ty));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            get_hardware_info().gpios[0].name, get_hardware_info().gpios[0].command_interfaces[27].name, &hw_vehicle_struct.command_state_.Tz));

        return command_interfaces;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::prepare_command_mode_switch(
        const std::vector<std::string> & /*start_interfaces*/,
        const std::vector<std::string> & /*stop_interfaces*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Command Mode Switch successful");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        reset_vehicle_simulation_state();
        commands_held_ = false;
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Activating... please wait...");

        publishStaticPoseTransform();

        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "System successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Deactivating... please wait...");
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        delta_seconds = period.seconds();
        // measurements
        casadi::DM y_k = casadi::DM::zeros(7, 1);
        {
            y_k(0) = hw_vehicle_struct.depth_from_pressure2;
            y_k(1) = hw_vehicle_struct.imu_state.roll_unwrap;
            y_k(2) = hw_vehicle_struct.imu_state.pitch_unwrap;
            y_k(3) = hw_vehicle_struct.imu_state.yaw_unwrap;
            y_k(4) = hw_vehicle_struct.dvl_state.vx;
            y_k(5) = hw_vehicle_struct.dvl_state.vy;
            y_k(6) = hw_vehicle_struct.dvl_state.vz;
        };

        // Wrap time step in a DM object.
        casadi::DM dt_dm(delta_seconds);
        std::vector<casadi::DM> ekf_inputs = {x_est_, P_est_, dt_dm, y_k, Q_, R_};

        // Call your CasADi function
        std::vector<casadi::DM> state_est = utils_service.uv_Exkalman_update(ekf_inputs);

        // casadi::DM u_uv_k = casadi::DM::zeros(6, 1);
        // {
        // u_uv_k(0) = hw_vehicle_struct.command_state_.Fx;
        // u_uv_k(1) = hw_vehicle_struct.command_state_.Fy;
        // u_uv_k(2) = hw_vehicle_struct.command_state_.Fz;
        // u_uv_k(3) = hw_vehicle_struct.command_state_.Tx;
        // u_uv_k(4) = hw_vehicle_struct.command_state_.Ty;
        // u_uv_k(5) = hw_vehicle_struct.command_state_.Tz;
        // };

        // casadi::Slice first12(0,12);

        // // shrink state and covariance
        // casadi::DM x12 = x_est_(first12, Slice());
        // casadi::DM P12 = P_est_(first12, first12);
        // casadi::DM Q12 = Q_(first12, first12);

        // std::vector<casadi::DM> dynamic_ekf_inputs = {x12, u_uv_k, P12, dt_dm, y_k, Q12, R_};
        // std::vector<casadi::DM> state_pred = utils_service.uv_dynamic_Exkalman_update(dynamic_ekf_inputs);

        // Extract result
        x_est_ = state_est[0];
        P_est_ = state_est[1];
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
        }

        // // Convert x_est_ to std::vector<double> or just read from DM?
        std::vector<double> x_est_v = dense_vector(x_est_, 18);

        // Update the estimated state in your hardware vehicle struct
        hw_vehicle_struct.estimate_state_.position_x = x_est_v[0];
        hw_vehicle_struct.estimate_state_.position_y = x_est_v[1];
        hw_vehicle_struct.estimate_state_.position_z = x_est_v[2];
        hw_vehicle_struct.estimate_state_.setEuler(x_est_v[3], x_est_v[4], x_est_v[5]);
        hw_vehicle_struct.estimate_state_.u = x_est_v[6];
        hw_vehicle_struct.estimate_state_.v = x_est_v[7];
        hw_vehicle_struct.estimate_state_.w = x_est_v[8];
        hw_vehicle_struct.estimate_state_.p = x_est_v[9];
        hw_vehicle_struct.estimate_state_.q = x_est_v[10];
        hw_vehicle_struct.estimate_state_.r = x_est_v[11];
        hw_vehicle_struct.current_state_.du = x_est_v[12];
        hw_vehicle_struct.current_state_.dv = x_est_v[13];
        hw_vehicle_struct.current_state_.dw = x_est_v[14];
        hw_vehicle_struct.current_state_.dp = x_est_v[15];
        hw_vehicle_struct.current_state_.dq = x_est_v[16];
        hw_vehicle_struct.current_state_.dr = x_est_v[17];

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        if (commands_held_)
        {
            RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                *node_topics_interface_->get_clock(),
                2000,
                "[%s] vehicle commands currently held after reset",
                hw_vehicle_struct.robot_prefix.c_str());
            hw_vehicle_struct.command_state_ = hw_vehicle_struct.default_state_;
            hw_vehicle_struct.sim_time = time_seconds;
            hw_vehicle_struct.sim_period = delta_seconds;
            for (auto &thruster : hw_vehicle_struct.hw_thrust_structs_)
            {
                thruster.command_state_.command_pwm = thruster.neutral_pwm;
                thruster.current_state_.sim_time = time_seconds;
                thruster.current_state_.sim_period = delta_seconds;
                thruster.current_state_.rc_pwm = thruster.neutral_pwm;
            }
            control_power_ = 0.0;
        }

        const DM thrust_config = thrust_configuration_dm(thrust_configuration_matrix_);

        if (hw_vehicle_struct.use_pwm)
        {
            std::vector<double> pwm_command_values;
            pwm_command_values.reserve(hw_vehicle_struct.hw_thrust_structs_.size());

            for (const auto &thruster : hw_vehicle_struct.hw_thrust_structs_)
            {
                pwm_command_values.push_back(thruster.command_state_.command_pwm);
            }

            DM pwm_commands = DM(pwm_command_values);

            std::vector<DM> thrusts_commands = utils_service.pwm_to_thrusts(pwm_commands);
            std::vector<double> thrusts_commands_doubles = dense_vector(thrusts_commands.at(0), 8);
            std::vector<DM> thrust2bodyf_args = {thrust_config, thrusts_commands_doubles};
            std::vector<DM> body_forces_command_resp = utils_service.thruster_f2body_f(thrust2bodyf_args);
            std::vector<double> body_forces_command = dense_vector(body_forces_command_resp.at(0), 6);

            hw_vehicle_struct.command_state_.Fx = body_forces_command[0];
            hw_vehicle_struct.command_state_.Fy = body_forces_command[1];
            hw_vehicle_struct.command_state_.Fz = body_forces_command[2];
            hw_vehicle_struct.command_state_.Tx = body_forces_command[3];
            hw_vehicle_struct.command_state_.Ty = body_forces_command[4];
            hw_vehicle_struct.command_state_.Tz = body_forces_command[5];
        }

        // IMPORTANT, clear per cycle
        uv_state.clear();
        uv_state.reserve(12);

        uv_input.clear();
        uv_input.reserve(6);

        uv_state.push_back(hw_vehicle_struct.current_state_.position_x);
        uv_state.push_back(hw_vehicle_struct.current_state_.position_y);
        uv_state.push_back(hw_vehicle_struct.current_state_.position_z);
        uv_state.push_back(hw_vehicle_struct.current_state_.roll);
        uv_state.push_back(hw_vehicle_struct.current_state_.pitch);
        uv_state.push_back(hw_vehicle_struct.current_state_.yaw);

        uv_state.push_back(hw_vehicle_struct.current_state_.u);
        uv_state.push_back(hw_vehicle_struct.current_state_.v);
        uv_state.push_back(hw_vehicle_struct.current_state_.w);
        uv_state.push_back(hw_vehicle_struct.current_state_.p);
        uv_state.push_back(hw_vehicle_struct.current_state_.q);
        uv_state.push_back(hw_vehicle_struct.current_state_.r);

        uv_input.push_back(hw_vehicle_struct.command_state_.Fx);
        uv_input.push_back(hw_vehicle_struct.command_state_.Fy);
        uv_input.push_back(hw_vehicle_struct.command_state_.Fz);
        uv_input.push_back(hw_vehicle_struct.command_state_.Tx);
        uv_input.push_back(hw_vehicle_struct.command_state_.Ty);
        uv_input.push_back(hw_vehicle_struct.command_state_.Tz);

        // Body velocity vector
        const double surge = hw_vehicle_struct.current_state_.u;
        const double sway  = hw_vehicle_struct.current_state_.v;
        const double heave = hw_vehicle_struct.current_state_.w;
        const double roll_rate  = hw_vehicle_struct.current_state_.p;
        const double pitch_rate = hw_vehicle_struct.current_state_.q;
        const double yaw_rate   = hw_vehicle_struct.current_state_.r;

        // Commanded body wrench
        const double Fx = hw_vehicle_struct.command_state_.Fx;
        const double Fy = hw_vehicle_struct.command_state_.Fy;
        const double Fz = hw_vehicle_struct.command_state_.Fz;
        const double Tx = hw_vehicle_struct.command_state_.Tx;
        const double Ty = hw_vehicle_struct.command_state_.Ty;
        const double Tz = hw_vehicle_struct.command_state_.Tz;

        // Instantaneous mechanical power
        control_power_ =
            std::abs(Fx * surge) +
            std::abs(Fy * sway) +
            std::abs(Fz * heave) +
            std::abs(Tx * roll_rate) +
            std::abs(Ty * pitch_rate) +
            std::abs(Tz * yaw_rate);

        // Energy accumulations
        control_energy_      += control_power_ * delta_seconds;

        std::vector<DM> FTinputs = {thrust_config, uv_input};
        std::vector<DM> thrust_outputs = utils_service.genForces2propThrust(FTinputs);
        std::vector<double> thrusts = dense_vector(thrust_outputs.at(0), 8);

        std::vector<DM> thrust2rads_args = {thrusts};
        std::vector<DM> thruster_rads_outputs = utils_service.thrust2rads(thrust2rads_args);
        std::vector<double> thrusts_rads_double = dense_vector(thruster_rads_outputs.at(0), 8);

        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position = hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position + thrusts_rads_double[i] * delta_seconds;
        }

        // Pull the latest external contact wrench from the contact pipeline.
        // The value is already in vehicle-body dynamics convention.
        std::array<double, 6> wrench_copy;
        {
            std::lock_guard<std::mutex> lock(contact_wrench_mutex_);
            wrench_copy = contact_wrench_body_;
        }

        //         // Debug print the external wrench that would be applied to dynamics
        // RCLCPP_INFO(
        //     rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
        //     "contact wrench body frame Fx=%.3f Fy=%.3f Fz=%.3f Tx=%.3f Ty=%.3f Tz=%.3f  dt=%.4f",
        //     wrench_copy[0],
        //     wrench_copy[1],
        //     wrench_copy[2],
        //     wrench_copy[3],
        //     wrench_copy[4],
        //     wrench_copy[5],
        //     delta_seconds
        // );

        // Build arm_base_f_ext from wrench_copy
        arm_base_f_ext.clear();
        arm_base_f_ext.reserve(6);
        arm_base_f_ext.push_back(wrench_copy[0]); // Fx body
        arm_base_f_ext.push_back(wrench_copy[1]); // Fy body
        arm_base_f_ext.push_back(wrench_copy[2]); // Fz body
        arm_base_f_ext.push_back(wrench_copy[3]); // Tx body
        arm_base_f_ext.push_back(wrench_copy[4]); // Ty body
        arm_base_f_ext.push_back(wrench_copy[5]); // Tz body

        // arm_base_f_ext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        vehicle_simulate_argument = {uv_state, uv_input, vehicle_parameters_new, delta_seconds, arm_base_f_ext};
        vehicle_sim = utils_service.vehicle_dynamics(vehicle_simulate_argument);
        vehicle_next_states = dense_vector(vehicle_sim.at(0), 12);

        hw_vehicle_struct.current_state_.position_x = vehicle_next_states[0];
        hw_vehicle_struct.current_state_.position_y = vehicle_next_states[1];
        hw_vehicle_struct.current_state_.position_z = vehicle_next_states[2];

        hw_vehicle_struct.depth_from_pressure2 = vehicle_next_states[2];

        hw_vehicle_struct.current_state_.setEuler(vehicle_next_states[3],
                                                  vehicle_next_states[4],
                                                  vehicle_next_states[5]);

        hw_vehicle_struct.imu_state.setEuler(vehicle_next_states[3],
                                             -vehicle_next_states[4],
                                             -vehicle_next_states[5]);

        hw_vehicle_struct.imu_state.roll_unwrap = vehicle_next_states[3];
        hw_vehicle_struct.imu_state.pitch_unwrap = -vehicle_next_states[4];
        hw_vehicle_struct.imu_state.yaw_unwrap = -vehicle_next_states[5];

        hw_vehicle_struct.current_state_.u = vehicle_next_states[6];
        hw_vehicle_struct.current_state_.v = vehicle_next_states[7];
        hw_vehicle_struct.current_state_.w = vehicle_next_states[8];
        hw_vehicle_struct.current_state_.p = vehicle_next_states[9];
        hw_vehicle_struct.current_state_.q = vehicle_next_states[10];
        hw_vehicle_struct.current_state_.r = vehicle_next_states[11];

        hw_vehicle_struct.dvl_state.vx = vehicle_next_states[6];
        hw_vehicle_struct.dvl_state.vy = vehicle_next_states[7];
        hw_vehicle_struct.dvl_state.vz = vehicle_next_states[8];

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.sim_time = time_seconds;
        hw_vehicle_struct.sim_period = delta_seconds;

        auto cam_msg_ptr_ptr = camera_mount_pitch_msg_buffer_.readFromRT();
        if (cam_msg_ptr_ptr && *cam_msg_ptr_ptr)
        {
            std_msgs::msg::Float32::SharedPtr cam_msg_ptr = *cam_msg_ptr_ptr;
            hw_vehicle_struct.camera_mountPitch_pwm += cam_msg_ptr->data;
            hw_vehicle_struct.camera_mountPitch_pwm =
                std::clamp(hw_vehicle_struct.camera_mountPitch_pwm, 1100.0, 1900.0);
            updateCameraMountPitchState();
        }

        publishRealtimePoseTransform();
        return hardware_interface::return_type::OK;
    }

    void SimVehicleSystemMultiInterfaceHardware::updateCameraMountPitchState()
    {
        hw_vehicle_struct.camera_mount_pitch_position =
            camera_mount_pwm_to_pitch(hw_vehicle_struct.camera_mountPitch_pwm);
    }

    void SimVehicleSystemMultiInterfaceHardware::cameraMountPitch_callback(
        const std_msgs::msg::Float32::SharedPtr msg)
    {
        camera_mount_pitch_msg_buffer_.writeFromNonRT(msg);
    }

    void SimVehicleSystemMultiInterfaceHardware::publishStaticPoseTransform()
    {
        // Capture the current time
        rclcpp::Time current_time = node_topics_interface_->now();

        // Create and send the static map transform
        geometry_msgs::msg::TransformStamped static_map_transform;

        static_map_transform.header.stamp = current_time;
        static_map_transform.header.frame_id = hw_vehicle_struct.world_frame_id;
        static_map_transform.child_frame_id = hw_vehicle_struct.map_frame_id;

        // Set translation based on current state
        static_map_transform.transform.translation.x = map_position_x;
        static_map_transform.transform.translation.y = map_position_y;
        static_map_transform.transform.translation.z = map_position_z;

        // Set rotation based on current state (quaternion)
        static_map_transform.transform.rotation.x = map_orientation_x;
        static_map_transform.transform.rotation.y = map_orientation_y;
        static_map_transform.transform.rotation.z = map_orientation_z;
        static_map_transform.transform.rotation.w = map_orientation_w;
        // Publish the static transform
        static_tf_broadcaster_->sendTransform(static_map_transform);

        // Create and send the static dvl transform
        geometry_msgs::msg::TransformStamped static_dvl_transform;

        static_dvl_transform.header.stamp = current_time;
        static_dvl_transform.header.frame_id = hw_vehicle_struct.body_frame_id;
        static_dvl_transform.child_frame_id = hw_vehicle_struct.robot_prefix + "dvl_link";

        // Set translation based on current state
        static_dvl_transform.transform.translation.x = -0.060;
        static_dvl_transform.transform.translation.y = 0.000;
        static_dvl_transform.transform.translation.z = -0.105;

        // Rotate the pose about X UPRIGHT
        q_rot_dvl.setRPY(0.0, 0.0, 0.0);

        q_rot_dvl.normalize();

        static_dvl_transform.transform.rotation.x = q_rot_dvl.x();
        static_dvl_transform.transform.rotation.y = q_rot_dvl.y();
        static_dvl_transform.transform.rotation.z = q_rot_dvl.z();
        static_dvl_transform.transform.rotation.w = q_rot_dvl.w();

        // Publish the static transform
        static_tf_broadcaster_->sendTransform(static_dvl_transform);

        // Create and send the static mocap markers transform
        geometry_msgs::msg::TransformStamped static_mocap_transform;

        static_mocap_transform.header.stamp = current_time;
        static_mocap_transform.header.frame_id = hw_vehicle_struct.map_frame_id;
        static_mocap_transform.child_frame_id = hw_vehicle_struct.robot_prefix + "mocap_link";

        // Set translation based on current state
        static_mocap_transform.transform.translation.x = -0.170;
        static_mocap_transform.transform.translation.y = 0.000;
        static_mocap_transform.transform.translation.z = 0.200;

        // Rotate the pose about X UPRIGHT
        q_rot_mocap.setRPY(0.0, 0.0, 0.0);

        q_rot_mocap.normalize();

        static_mocap_transform.transform.rotation.x = q_rot_mocap.x();
        static_mocap_transform.transform.rotation.y = q_rot_mocap.y();
        static_mocap_transform.transform.rotation.z = q_rot_mocap.z();
        static_mocap_transform.transform.rotation.w = q_rot_mocap.w();

        // Publish the static transform
        static_tf_broadcaster_->sendTransform(static_mocap_transform);
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "Published static odom transform once during activation.");
    };

    void SimVehicleSystemMultiInterfaceHardware::publishRealtimePoseTransform()
    {
        rclcpp::Time current_time = node_topics_interface_->now();
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            auto &transforms = realtime_transform_publisher_->msg_.transforms;
            if (transforms.size() < 2)
            {
                transforms.resize(2);
            }
            auto &StateEstimateTransform = transforms.front();
            StateEstimateTransform.header.frame_id = hw_vehicle_struct.map_frame_id;
            StateEstimateTransform.child_frame_id = hw_vehicle_struct.body_frame_id;
            StateEstimateTransform.header.stamp = current_time;
            StateEstimateTransform.transform.translation.x = hw_vehicle_struct.estimate_state_.position_x;
            StateEstimateTransform.transform.translation.y = -hw_vehicle_struct.estimate_state_.position_y;
            StateEstimateTransform.transform.translation.z = -hw_vehicle_struct.estimate_state_.position_z;

            q_orig.setW(hw_vehicle_struct.estimate_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct.estimate_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct.estimate_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct.estimate_state_.orientation_z);

            q_orig.normalize();

            // get roll/pitch/yaw
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

            // invert yaw only
            yaw = -yaw;
            pitch = -pitch;

            // rebuild
            tf2::Quaternion q_fixed;
            q_fixed.setRPY(roll, pitch, yaw);
            q_fixed.normalize();

            StateEstimateTransform.transform.rotation.x = q_fixed.x();
            StateEstimateTransform.transform.rotation.y = q_fixed.y();
            StateEstimateTransform.transform.rotation.z = q_fixed.z();
            StateEstimateTransform.transform.rotation.w = q_fixed.w();

            auto &camera_mount_transform = transforms[1];
            camera_mount_transform.header.frame_id = hw_vehicle_struct.body_frame_id;
            camera_mount_transform.child_frame_id =
                hw_vehicle_struct.robot_prefix + "camera_mount_link";
            camera_mount_transform.header.stamp = current_time;
            camera_mount_transform.transform.translation.x = 0.21;
            camera_mount_transform.transform.translation.y = 0.0;
            camera_mount_transform.transform.translation.z = 0.067;

            tf2::Quaternion camera_mount_rotation;
            camera_mount_rotation.setRPY(0.0, hw_vehicle_struct.camera_mount_pitch_position, 0.0);
            camera_mount_rotation.normalize();
            camera_mount_transform.transform.rotation.x = camera_mount_rotation.x();
            camera_mount_transform.transform.rotation.y = camera_mount_rotation.y();
            camera_mount_transform.transform.rotation.z = camera_mount_rotation.z();
            camera_mount_transform.transform.rotation.w = camera_mount_rotation.w();

            // Publish the TF
            realtime_transform_publisher_->unlockAndPublish();
        }
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimVehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
