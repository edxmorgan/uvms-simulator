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
#include <Eigen/Cholesky>
#include <cstdlib>
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
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"

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

        ros2_control_blue_reach_5::msg::SimVehicleDynamics default_vehicle_dynamics()
        {
            auto msg = ros2_control_blue_reach_5::msg::SimVehicleDynamics();
            const auto values = default_nominal_vehicle_parameters();
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
        for (int i = 12; i < 18; ++i) P_est_(i, i) = 1.0;
        estimated_disturbance_.fill(0.0);
        estimator_applied_input_ = casadi::DM::zeros(6, 1);
        estimator_step_ = 0.0;
        estimator_valid_ = 0.0;
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
            if (i >= 12) for (int j=0;j<6;++j)
                disturbance_covariance_[6*(i-12)+j]=double(P_est_(i,12+j));
        }
        restore_nominal_estimator_model();
        pending_retained_model_.reset();
        if (node_topics_interface_) estimator_model_reset_stamp_ = node_topics_interface_->now();
    }

    void SimVehicleSystemMultiInterfaceHardware::restore_nominal_estimator_model()
    {
        noise_adaptation_.reset(Q_);
        ++evaluation_reset_sequence_;
        const auto parameters = pack_vehicle_dynamics(default_vehicle_dynamics());
        estimator_nominal_parameters_ = casadi::DM(parameters);
        std::copy(parameters.begin(), parameters.end(), estimator_model_readback_.begin());
        estimator_model_retained_ = 0.;
        estimator_model_epoch_ = estimator_model_accepted_time_ = -1.;
        estimator_model_net_weight_only_ = false;
        for (int i = 12; i < 18; ++i) {
            x_est_(i) = 0.;
            for (int j = 0; j < 18; ++j) P_est_(i,j) = P_est_(j,i) = 0.;
            P_est_(i,i) = 1.;
        }
        estimated_disturbance_.fill(0.);
    }

    void SimVehicleSystemMultiInterfaceHardware::receive_retained_model(const Identification &m)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        const auto stamp = rclcpp::Time(m.stamp);
        const double age = (node_topics_interface_->now() - stamp).seconds();
        if (m.hardware_measurements || m.joint != hw_vehicle_struct.robot_prefix + "IOs" ||
            !m.retained_available || !m.retained_valid ||
            m.source_generation != sim_generation_ || m.epoch >= (1ULL << 53) ||
            stamp < estimator_model_reset_stamp_ || age < 0. || age > 3. ||
            !std::isfinite(m.retained_accepted_time) || m.retained_accepted_time < 0. ||
            m.retained_accepted_time > hw_vehicle_struct.sim_time) return;
        if (estimator_model_retained_ == 1. &&
            m.retained_accepted_time < estimator_model_accepted_time_) return;
        const auto &p = m.retained_parameters;
        for (double v : p) if (!std::isfinite(v)) return;
        if (std::abs(p[3]-p[6]) > 1e-8 || std::abs(p[4]-p[5]) > 1e-8 ||
            p[0]<=0 || p[1]<=0 || p[2]<=0 || p[7]<=0 || p[8]<=0 || p[9]<=0 ||
            p[0]*p[8]-p[3]*p[3]<=0 || p[1]*p[7]-p[4]*p[4]<=0 ||
            p[10]<=0 || p[11]<0) return;
        for (int i=15; i<27; ++i) if (p[i]>0.) return;
        pending_retained_model_ = m;
        retained_model_received_ = std::chrono::steady_clock::now();
    }

    void SimVehicleSystemMultiInterfaceHardware::apply_retained_model()
    {
        // Net weight is sufficient only in the identified submerged/surface regime.
        const bool above_water = hw_vehicle_struct.depth_from_pressure2 < 0. || double(x_est_(2)) < 0.;
        if (estimator_model_net_weight_only_ && above_water) restore_nominal_estimator_model();
        if (!pending_retained_model_) return;
        const auto m = *pending_retained_model_;
        pending_retained_model_.reset();
        if (m.source_generation != sim_generation_ ||
            std::chrono::duration<double>(std::chrono::steady_clock::now()-retained_model_received_).count()>3. ||
            (m.retained_net_weight_only && above_water) ||
            (estimator_model_retained_ == 1. && estimator_model_epoch_ == m.epoch &&
             estimator_model_accepted_time_ == m.retained_accepted_time)) return;
        // Preserve navigation state/covariance, but discard disturbance attributed to the old model.
        restore_nominal_estimator_model();
        for (int i=0; i<27; ++i) {
            estimator_nominal_parameters_(i) = m.retained_parameters[i];
            estimator_model_readback_[i] = m.retained_parameters[i];
        }
        estimator_model_retained_ = 1.;
        estimator_model_epoch_ = m.epoch;
        estimator_model_accepted_time_ = m.retained_accepted_time;
        estimator_model_net_weight_only_ = m.retained_net_weight_only;
    }

    void SimVehicleSystemMultiInterfaceHardware::seed_vehicle_estimator_from_state(
        const blue::dynamics::Vehicle::Pose_vel &state)
    {
        x_est_ = casadi::DM::zeros(18, 1);
        x_est_(0) = state.position_x;
        x_est_(1) = state.position_y;
        x_est_(2) = state.position_z;
        x_est_(3) = state.roll;
        x_est_(4) = state.pitch;
        x_est_(5) = state.yaw;
        x_est_(6) = state.u;
        x_est_(7) = state.v;
        x_est_(8) = state.w;
        x_est_(9) = state.p;
        x_est_(10) = state.q;
        x_est_(11) = state.r;
        // Simulator reset supplies the initial navigation prior, never a wrench prior.
        // Disturbance states start at zero regardless of the requested plant wrench.
    }

    void SimVehicleSystemMultiInterfaceHardware::seed_vehicle_sensor_state_from_state(
        const blue::dynamics::Vehicle::Pose_vel &state)
    {
        hw_vehicle_struct.depth_from_pressure2 = state.position_z;

        hw_vehicle_struct.imu_state.setEuler(state.roll, -state.pitch, -state.yaw);
        hw_vehicle_struct.imu_state.roll_unwrap = state.roll;
        hw_vehicle_struct.imu_state.pitch_unwrap = -state.pitch;
        hw_vehicle_struct.imu_state.yaw_unwrap = -state.yaw;
        hw_vehicle_struct.imu_state.angular_vel_x = state.p;
        hw_vehicle_struct.imu_state.angular_vel_y = -state.q;
        hw_vehicle_struct.imu_state.angular_vel_z = -state.r;
        hw_vehicle_struct.imu_state.linear_acceleration_x = state.du;
        hw_vehicle_struct.imu_state.linear_acceleration_y = state.dv;
        hw_vehicle_struct.imu_state.linear_acceleration_z = state.dw;

        hw_vehicle_struct.dvl_state.vx = state.u;
        hw_vehicle_struct.dvl_state.vy = state.v;
        hw_vehicle_struct.dvl_state.vz = state.w;
    }

    void SimVehicleSystemMultiInterfaceHardware::reset_vehicle_simulation_state()
    {
        sim_step_ = 0.0;
        sim_generation_ += 1.0;
        sensor_elapsed_ = 0.0;
        sensor_next_sample_.fill(0.0);
        sensor_sample_time_.fill(0.0);
        sensor_new_.fill(0.0);
        raw_imu_new_.fill(0.0);
        raw_imu_measurement_.fill(0.0);
        sensor_measurement_.fill(0.0);
        sensor_noise_rng_.seed(sensor_noise_seed_);
        sensor_normal_.reset();
        hw_vehicle_struct.current_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.async_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.command_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.estimate_state_ = hw_vehicle_struct.default_state_;
        hw_vehicle_struct.dvl_state = {};
        hw_vehicle_struct.imu_state = {};
        seed_vehicle_sensor_state_from_state(hw_vehicle_struct.default_state_);
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

        if (disturbance_receiver_) disturbance_receiver_->reset();
        commands_held_ = true;
        reset_vehicle_estimators();
        seed_vehicle_estimator_from_state(hw_vehicle_struct.default_state_);
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
            seed_vehicle_sensor_state_from_state(requested_state);
            seed_vehicle_estimator_from_state(requested_state);
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

    void SimVehicleSystemMultiInterfaceHardware::apply_pending_reset_request()
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
            reset_vehicle_simulation_state(*request);
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
            reset_failure_message_ = "unknown exception while applying vehicle reset";
            reset_failed_sequence_ = sequence;
            reset_applied_sequence_ = sequence;
        }
        reset_applied_cv_.notify_all();
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
        utils_service.uv_Exkalman_update = utils_service.load_casadi_fun("disturbance_ekf_update", "libdisturbanceEKF_next.so");
        estimator_predict_ = utils_service.load_casadi_fun("disturbance_ekf_predict", "libdisturbanceEKF_next.so");
        estimator_covariance_predict_ = utils_service.load_casadi_fun("disturbance_ekf_predict_covariance", "libdisturbanceEKF_next.so");
        estimator_correct_ = utils_service.load_casadi_fun("disturbance_ekf_correct", "libdisturbanceEKF_next.so");
        sensor_correct_ = utils_service.load_casadi_fun("disturbance_ekf_sensor_correct", "libdisturbanceEKF_next.so");
        physical_sensor_model_ = utils_service.load_casadi_fun("vehicle_sensor_model", "libdisturbanceEKF_next.so");
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
        estimator_nominal_parameters_ = casadi::DM(pack_vehicle_dynamics(default_dynamics));
        const auto estimator_model = pack_vehicle_dynamics(default_dynamics);
        std::copy(estimator_model.begin(), estimator_model.end(), estimator_model_readback_.begin());
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
            // Includes simulation step metadata and disturbance estimator diagnostics.
            if (gpio.state_interfaces.size() != 206)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 206 expected.", gpio.name.c_str(),
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
            configure_estimator_noise();
            retained_model_subscription_ = node_topics_interface_->create_subscription<Identification>(
                "/vehicle_identification/" + hw_vehicle_struct.robot_prefix + "IOs/candidate", 10,
                [this](Identification::ConstSharedPtr m) { receive_retained_model(*m); });

            // Opt-in simulation evaluation telemetry. Never changes estimator inputs or outputs.
            if (std::getenv("UVMS_EKF_EVALUATION_TRACE")) {
                evaluation_trace_pub_ = node_topics_interface_->create_publisher<std_msgs::msg::Float64MultiArray>(
                    "/" + hw_vehicle_struct.robot_prefix + "ekf_evaluation_trace", rclcpp::QoS(2048).reliable());
            }

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

            disturbance_receiver_ = std::make_shared<DisturbanceReceiver>(
                node_topics_interface_, hw_vehicle_struct.robot_prefix, "vehicle");
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

        sensor_noise_service_ = node_topics_interface_->create_service<std_srvs::srv::SetBool>(
            "/" + hw_vehicle_struct.robot_prefix + "set_sensor_noise",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                sensor_noise_requested_.store(request->data);
                response->success = true;
                response->message = "Sensor-noise change queued for next simulation step; EKF tuning unchanged";
            });

        auto reset_state_callback =
            [this](
                const std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Request> request,
                std::shared_ptr<ros2_control_blue_reach_5::srv::ResetSimUvms::Response> response)
        {
            if (request->set_vehicle_dynamics &&
                !valid_thrust_configuration_matrix(request->vehicle_dynamics.thrust_configuration_matrix))
            {
                response->success = false;
                response->message = "vehicle_dynamics.thrust_configuration_matrix must contain a nonzero 6x8 row-major matrix";
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
                response->message = "timeout applying simulated vehicle reset";
                return;
            }
            if (reset_failed_sequence_ == sequence)
            {
                response->success = false;
                response->message = reset_failure_message_;
                return;
            }
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
        noise_adaptation_.stop();
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

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "sim_step", &sim_step_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "sim_generation", &sim_generation_));
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name, "estimator_model_retained", &estimator_model_retained_);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name, "estimator_model_epoch", &estimator_model_epoch_);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name, "estimator_model_accepted_time", &estimator_model_accepted_time_);
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
        for (std::size_t i=0; i<3; ++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "sensor_sample_time." + std::to_string(i), &sensor_sample_time_[i]);
        for (std::size_t i=0; i<7; ++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "sensor_new." + std::to_string(i), &sensor_new_[i]);
        for (std::size_t i=0; i<6; ++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "sensor_new." + std::to_string(7+i), &raw_imu_new_[i]);
        const std::array<std::string, 6> wrench_axes = {
            "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
        for (std::size_t i = 0; i < estimator_model_readback_.size(); ++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimator_nominal_parameter." + std::to_string(i), &estimator_model_readback_[i]);
        for (std::size_t i = 0; i < wrench_axes.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimated_disturbance." + wrench_axes[i],
                &estimated_disturbance_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                get_hardware_info().gpios[0].name, "estimated_disturbance.variance." + wrench_axes[i],
                &P_diag_[12 + i]));
        }
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "estimator_step", &estimator_step_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            get_hardware_info().gpios[0].name, "estimator_valid", &estimator_valid_));
        for (std::size_t i=0;i<6;++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimator_noise.intensity."+std::to_string(i), &noise_adaptation_.active_density[i]);
        const std::array<std::string,6> noise_status_names{
            "enabled", "applied_fit_count", "dropped", "failed", "fit_seconds", "endpoint_step"};
        for (std::size_t i=0;i<noise_status_names.size();++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimator_noise."+noise_status_names[i], &noise_adaptation_.diagnostics[i]);
        for (std::size_t i=0;i<6;++i) for (std::size_t j=0;j<6;++j)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimated_disturbance.covariance."+std::to_string(i)+"."+std::to_string(j),
                &disturbance_covariance_[6*i+j]);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "sensor_noise_enabled", &sensor_noise_enabled_);
        // Posterior model acceleration; keep physical/raw acceleration separately.
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "linear_acceleration_estimate.x", &hw_vehicle_struct.estimate_state_.du);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "linear_acceleration_estimate.y", &hw_vehicle_struct.estimate_state_.dv);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "linear_acceleration_estimate.z", &hw_vehicle_struct.estimate_state_.dw);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "angular_acceleration_estimate.x", &hw_vehicle_struct.estimate_state_.dp);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "angular_acceleration_estimate.y", &hw_vehicle_struct.estimate_state_.dq);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
            "angular_acceleration_estimate.z", &hw_vehicle_struct.estimate_state_.dr);
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
        noise_adaptation_.reset(Q_);
        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Deactivating... please wait...");
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void SimVehicleSystemMultiInterfaceHardware::configure_estimator_noise()
    {
        // Sample at independent sensor rates on the completed dynamics clock.
        // EKF uncertainty is configured independently of generated sensor noise.
        // Unknown acceleration remains represented by the augmented wrench.
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        sensor_noise_requested_.store(node_topics_interface_->declare_parameter<bool>(
            "sim_sensor_noise_enabled", true, descriptor));
        sensor_noise_enabled_ = sensor_noise_requested_.load() ? 1.0 : 0.0;

        auto diagonal = [&](const char *name, std::size_t size, double value, bool positive) {
            const auto values = node_topics_interface_->declare_parameter<std::vector<double>>(
                name, std::vector<double>(size, value), descriptor);
            if (values.size() != size || std::any_of(values.begin(), values.end(),
                [positive](double v) { return !std::isfinite(v) || v < 0.0 || (positive && v == 0.0); }))
                throw std::invalid_argument(std::string(name) + ": invalid covariance diagonal");
            return casadi::DM::diag(casadi::DM(values));
        };
        const auto navigation = diagonal("ekf_navigation_process_intensities", 12, 0.0, false);
        // Separate physical units: force N^2/s, moment (N m)^2/s. This is a
        // responsive example prior, not a calibration or an injected-load input.
        const auto wrench_values = node_topics_interface_->declare_parameter<std::vector<double>>(
            "ekf_wrench_process_intensities", {7e-4,7e-4,7e-4,7e-4,7e-4,7e-4}, descriptor);
        if (wrench_values.size()!=6 || std::any_of(wrench_values.begin(),wrench_values.end(),
            [](double v){return !std::isfinite(v)||v<0.;}))
            throw std::invalid_argument("ekf_wrench_process_intensities requires six nonnegative values");
        const auto full_wrench=node_topics_interface_->declare_parameter<std::vector<double>>(
            "ekf_wrench_process_covariance", std::vector<double>{}, descriptor);
        const auto wrench=casadi_reach_alpha_5::wrench_process_covariance(full_wrench,wrench_values);
        Q_ = casadi::DM::diagcat({navigation, wrench});
        const bool adapt_noise = node_topics_interface_->declare_parameter<bool>(
            "ekf_adapt_wrench_process_noise", false, descriptor);
        if (adapt_noise && !full_wrench.empty())
            throw std::invalid_argument("ekf_adapt_wrench_process_noise does not support a full covariance override");
        VehicleDisturbanceNoiseAdaptation::Density configured_noise{};
        for (int i=0;i<6;++i) configured_noise[i]=double(wrench(i,i));
        noise_adaptation_.configure(configured_noise, adapt_noise
            ? std::string("libdisturbanceEKF_next.so") : std::string());

        const auto rates = node_topics_interface_->declare_parameter<std::vector<double>>(
            "sim_sensor_rates_hz", {10.0, 200.0, 10.0}, descriptor);
        if (rates.size() != 3 || std::any_of(rates.begin(), rates.end(), [](double v) {
            return !std::isfinite(v) || v < 0.0 || v > 1000.0;
        })) throw std::invalid_argument("sim_sensor_rates_hz: expected three rates in [0,1000]; zero means every step");
        std::copy(rates.begin(), rates.end(), sensor_rates_.begin());
        const auto standard_deviations = node_topics_interface_->declare_parameter<std::vector<double>>(
            "sim_sensor_noise_std", std::vector<double>(7, 0.001), descriptor);
        if (standard_deviations.size() != 7 || std::any_of(
            standard_deviations.begin(), standard_deviations.end(), [](double v) {
                return !std::isfinite(v) || v < 0.0 || !std::isfinite(v*v);
            })) throw std::invalid_argument("sim_sensor_noise_std: expected seven finite nonnegative standard deviations");
        std::copy(standard_deviations.begin(), standard_deviations.end(), sensor_noise_std_.begin());
        // Stable per-robot default: identical robots must not share noise streams.
        uint32_t default_seed = 2166136261u;
        for (unsigned char c : hw_vehicle_struct.robot_prefix) default_seed = (default_seed ^ c) * 16777619u;
        const auto seed = node_topics_interface_->declare_parameter<int64_t>(
            "sim_sensor_noise_seed", default_seed, descriptor);
        if (seed < 0 || seed > std::numeric_limits<uint32_t>::max())
            throw std::invalid_argument("sim_sensor_noise_seed: expected an unsigned 32-bit seed");
        sensor_noise_seed_ = static_cast<uint32_t>(seed);
        sensor_noise_rng_.seed(sensor_noise_seed_);
        sensor_normal_.reset();
        R_ = casadi::DM::zeros(7, 7);
        R_(casadi::Slice(0,4),casadi::Slice(0,4)) =
            diagonal("depth_attitude_measurement_variances", 4, 1e-6, true);
        const auto assumed_dvl = node_topics_interface_->declare_parameter<std::vector<double>>(
            "dvl_measurement_covariance", {1e-6,0.,0.,0.,1e-6,0.,0.,0.,1e-6}, descriptor);
        const auto checked_covariance = [](const std::vector<double>& values, const char* name) {
            if (values.size()!=9) throw std::invalid_argument(std::string(name)+": expected nine row-major entries");
            Eigen::Matrix3d C;
            for(int i=0;i<3;++i) for(int j=0;j<3;++j) C(i,j)=values[3*i+j];
            const double scale=C.cwiseAbs().maxCoeff();
            if(!C.allFinite() || scale<=0. ||
               (C-C.transpose()).cwiseAbs().maxCoeff()>1e-6*scale)
                throw std::invalid_argument(std::string(name)+": expected finite symmetric covariance");
            C=((C+C.transpose())*.5).eval();
            if(Eigen::LLT<Eigen::Matrix3d>(C).info()!=Eigen::Success)
                throw std::invalid_argument(std::string(name)+": covariance must be positive definite");
            return C;
        };
        const auto assumed_C=checked_covariance(assumed_dvl,"dvl_measurement_covariance");
        for(int i=0;i<3;++i) for(int j=0;j<3;++j) R_(4+i,4+j)=assumed_C(i,j);
        // Optional measured A50 covariance in body FRD, row-major (m/s)^2.
        // Empty keeps the diagonal std profile; explicit covariance takes precedence.
        const auto covariance = node_topics_interface_->declare_parameter<std::vector<double>>(
            "sim_dvl_covariance", std::vector<double>{}, descriptor);
        sensor_dvl_factor_.fill(0.0);
        for (int i=0; i<3; ++i) sensor_dvl_factor_[3*i+i] = sensor_noise_std_[4+i];
        if (!covariance.empty()) {
            const auto C=checked_covariance(covariance,"sim_dvl_covariance");
            Eigen::LLT<Eigen::Matrix3d> factor(C);
            const Eigen::Matrix3d L=factor.matrixL();
            for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
                sensor_dvl_factor_[3*i+j]=L(i,j);
            }
        }
        const auto raw_std=node_topics_interface_->declare_parameter<std::vector<double>>(
            "sim_raw_imu_noise_std", {.001,.001,.001,.01,.01,.01}, descriptor);
        if(raw_std.size()!=6 || std::any_of(raw_std.begin(),raw_std.end(),
            [](double v){return !std::isfinite(v)||v<0.||!std::isfinite(v*v);}))
            throw std::invalid_argument("sim_raw_imu_noise_std requires six finite nonnegative values");
        std::copy(raw_std.begin(),raw_std.end(),raw_imu_std_.begin());
        const auto offset=node_topics_interface_->declare_parameter<std::vector<double>>(
            "imu_offset_body", {0.,0.,0.}, descriptor);
        if(offset.size()!=3 || std::any_of(offset.begin(),offset.end(),[](double v){return !std::isfinite(v);}))
            throw std::invalid_argument("imu_offset_body requires three finite FRD coordinates");
        imu_offset_=casadi::DM(offset);
        sensor_gravity_=node_topics_interface_->declare_parameter<double>("sensor_gravity",9.80665,descriptor);
        if(!std::isfinite(sensor_gravity_)||sensor_gravity_<=0.)
            throw std::invalid_argument("sensor_gravity must be positive");
        sensor_R_=casadi::DM::eye(13);
        sensor_R_(casadi::Slice(0,7),casadi::Slice(0,7))=R_;
        const auto raw_variances=node_topics_interface_->declare_parameter<std::vector<double>>(
            "raw_imu_measurement_variances", {1e-6,1e-6,1e-6,1e-4,1e-4,1e-4}, descriptor);
        if(raw_variances.size()!=6 || std::any_of(raw_variances.begin(),raw_variances.end(),
            [](double v){return !std::isfinite(v)||v<=0.;}))
            throw std::invalid_argument("raw_imu_measurement_variances requires six finite positive variances");
        for(int i=0;i<6;++i) sensor_R_(7+i,7+i)=raw_variances[i];
    }

    std::array<double, 7> SimVehicleSystemMultiInterfaceHardware::sample_vehicle_measurements(
        const std::vector<double> &state, double dt)
    {
        if (!std::isfinite(dt) || dt <= 0.0)
            throw std::invalid_argument("sensor sample requires a positive completed-step duration");
        sensor_noise_enabled_ = sensor_noise_requested_.load() ? 1.0 : 0.0;
        sensor_elapsed_ += dt;
        sensor_new_.fill(0.0);
        constexpr std::array<int, 4> offsets{0, 1, 4, 7};
        for (std::size_t group=0; group<3; ++group) {
            const double rate = sensor_rates_[group];
            // Quantize the scheduled sample to the nearest completed step.
            // Waiting strictly until after each deadline loses samples when the
            // sensor and physics rates match but the wall-clock period jitters.
            const double horizon = sensor_elapsed_ + (rate > 0.0 ?
                std::min(0.5*dt, 0.5/rate) : 0.0);
            if (horizon + 1e-10 < sensor_next_sample_[group]) continue;
            std::array<double,3> dvl_noise{};
            if (group==2) for (auto &value:dvl_noise) value=sensor_normal_(sensor_noise_rng_);
            for (int i=offsets[group]; i<offsets[group+1]; ++i) {
                double noise=0.0;
                if (group==2) {
                    for (int j=0;j<3;++j) noise+=sensor_dvl_factor_[3*(i-4)+j]*dvl_noise[j];
                } else noise=sensor_noise_std_[i]*sensor_normal_(sensor_noise_rng_);
                sensor_measurement_[i] = state.at(i+2) + apply_sensor_noise(noise);
                sensor_new_[i] = 1.0;
            }
            sensor_sample_time_[group] = sensor_elapsed_;
            // No catch-up samples invented from the latest state after a long step.
            // Quantization is at most one physics step; schedules never accumulate drift.
            sensor_next_sample_[group] = rate == 0.0 ? sensor_elapsed_ :
                (std::floor((horizon + 1e-10)*rate) + 1.0)/rate;
        }
        return sensor_measurement_;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        delta_seconds = period.seconds();
        apply_pending_reset_request();
        // Each measurement belongs to the last completed write, not this read period.
        if (dynamics_paused_ || sim_step_ == estimator_step_)
        {
            return hardware_interface::return_type::OK;
        }
        casadi::DM y_k = casadi::DM::zeros(7, 1);
        {
            y_k(0) = hw_vehicle_struct.depth_from_pressure2;
            y_k(1) = hw_vehicle_struct.imu_state.roll_unwrap;
            y_k(2) = -hw_vehicle_struct.imu_state.pitch_unwrap;
            y_k(3) = -hw_vehicle_struct.imu_state.yaw_unwrap;
            y_k(4) = hw_vehicle_struct.dvl_state.vx;
            y_k(5) = hw_vehicle_struct.dvl_state.vy;
            y_k(6) = hw_vehicle_struct.dvl_state.vz;
        };

        const double completed_dt = hw_vehicle_struct.sim_period;
        if (sim_step_ != estimator_step_ + 1.0 ||
            !std::isfinite(completed_dt) || completed_dt <= 0.0)
        {
            noise_adaptation_.before_prediction(Q_, false);
            estimator_valid_ = 0.0;
            RCLCPP_ERROR(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                         "Disturbance EKF lost completed-step synchronization; reset required");
            return hardware_interface::return_type::ERROR;
        }
        apply_retained_model();
        noise_adaptation_.before_prediction(Q_, true);
        const auto raw_measurement = casadi::DM(std::vector<double>(raw_imu_measurement_.begin(),raw_imu_measurement_.end()));
        const auto y_sensor = casadi::DM::vertcat({y_k, raw_measurement});
        const auto sensor_mask = casadi::DM::vertcat({
            casadi::DM(std::vector<double>(sensor_new_.begin(),sensor_new_.end())),
            casadi::DM(std::vector<double>(raw_imu_new_.begin(),raw_imu_new_.end()))});
        std::vector<casadi::DM> ekf_inputs = {
            x_est_, P_est_, estimator_applied_input_, estimator_nominal_parameters_,
            completed_dt, y_sensor, Q_ * completed_dt, sensor_R_};
        std::vector<casadi::DM> state_est;
        try
        {
            const auto prediction = casadi_reach_alpha_5::predict_disturbance_covariance(
                estimator_covariance_predict_, x_est_, P_est_, estimator_applied_input_,
                estimator_nominal_parameters_, completed_dt, Q_);
            const auto &prior_P = prediction[1];
            // The trace stores the actual integrated discrete covariance.
            ekf_inputs[6] = prediction[3];
            const auto corrected = sensor_correct_(std::vector<casadi::DM>{
                prediction[0], prior_P, estimator_applied_input_, estimator_nominal_parameters_,
                y_sensor, sensor_R_, sensor_mask, 1.0, imu_offset_, sensor_gravity_});
            // Evaluate the nominal acceleration directly; do not run a second
            // zero-duration Kalman correction just to obtain this output.
            const auto model = physical_sensor_model_(std::vector<casadi::DM>{
                corrected[0], estimator_applied_input_, estimator_nominal_parameters_,
                imu_offset_, sensor_gravity_});
            state_est = {corrected[0],corrected[1],corrected[2],corrected[3],model[2]};
            for (const auto &output : state_est)
                for (const double value : output.nonzeros())
                    if (!std::isfinite(value)) throw std::runtime_error("nonfinite EKF output");
            noise_adaptation_.after_correction(prediction, corrected, completed_dt,
                static_cast<uint64_t>(sim_step_));
        }
        catch (const std::exception &error)
        {
            noise_adaptation_.before_prediction(Q_, false);
            estimator_valid_ = 0.0;
            RCLCPP_ERROR(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                         "Disturbance EKF failed: %s", error.what());
            return hardware_interface::return_type::ERROR;
        }
        if (evaluation_trace_pub_) {
            std_msgs::msg::Float64MultiArray trace;
            trace.data = {node_topics_interface_->now().seconds(), hw_vehicle_struct.sim_time,
                          sim_generation_, sim_step_, completed_dt, evaluation_reset_sequence_,
                          estimator_model_retained_, estimator_model_epoch_, estimator_model_accepted_time_};
            auto append = [&](const casadi::DM &value) {
                // Dense, column-major, including structural zeros in covariances.
                for (casadi_int j=0; j<value.size2(); ++j)
                    for (casadi_int i=0; i<value.size1(); ++i) trace.data.push_back(double(value(i,j)));
            };
            for (const auto &value : ekf_inputs) append(value);
            for (const auto &value : state_est) append(value);
            for (double value : vehicle_next_states) trace.data.push_back(value);
            for (const auto &value : vehicle_parameters_new) append(value);
            trace.data.insert(trace.data.end(), {hw_vehicle_struct.imu_state.angular_vel_x, hw_vehicle_struct.imu_state.angular_vel_y, hw_vehicle_struct.imu_state.angular_vel_z});
            for (double value : arm_base_f_ext) trace.data.push_back(value);
            // Schema 4 stores integrated Qd; 13 measurements/R/innovations and explicit mask.
            // Suffix: mask(13), sample times(3), clock(1), IMU offset(3), gravity(1).
            // Consumers of schema 1 must not replay held observations as fresh data.
            trace.layout.dim.resize(1);
            trace.layout.dim[0].label = "vehicle_ekf_trace_v4_integrated_process_noise";
            trace.data.insert(trace.data.end(), sensor_new_.begin(), sensor_new_.end());
            trace.data.insert(trace.data.end(), raw_imu_new_.begin(), raw_imu_new_.end());
            trace.data.insert(trace.data.end(), sensor_sample_time_.begin(), sensor_sample_time_.end());
            trace.data.push_back(sensor_elapsed_);
            append(imu_offset_);trace.data.push_back(sensor_gravity_);
            trace.layout.dim[0].size = trace.layout.dim[0].stride = trace.data.size();
            evaluation_trace_pub_->publish(trace);
        }
        estimator_step_ = sim_step_;
        estimator_valid_ = 1.0;

        // Extract result
        x_est_ = state_est[0];
        P_est_ = state_est[1];
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
            if (i >= 12) for (int j=0;j<6;++j)
                disturbance_covariance_[6*(i-12)+j]=double(P_est_(i,12+j));
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
        for (int i = 0; i < 6; ++i) estimated_disturbance_[i] = x_est_v[12 + i];
        const auto acceleration = dense_vector(state_est[4], 6);
        hw_vehicle_struct.estimate_state_.du = acceleration[0];
        hw_vehicle_struct.estimate_state_.dv = acceleration[1];
        hw_vehicle_struct.estimate_state_.dw = acceleration[2];
        hw_vehicle_struct.estimate_state_.dp = acceleration[3];
        hw_vehicle_struct.estimate_state_.dq = acceleration[4];
        hw_vehicle_struct.estimate_state_.dr = acceleration[5];

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        std::lock_guard<std::mutex> lock(simulation_state_mutex_);
        apply_pending_reset_request();
        // Reset clears cached timing; this write still owns its actual interval.
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

        const auto contact_only = wrench_copy;
        // Independent injection input: never overwrites physical contact.
        const auto injection = disturbance_receiver_ ? disturbance_receiver_->sample() : msg::DisturbanceCommand{};
        const auto injected_wrench = DisturbanceLease::wrench(injection);
        for (size_t i=0; i<6; ++i) wrench_copy[i] += injected_wrench[i];
        // Build arm_base_f_ext from wrench_copy
        arm_base_f_ext.clear();
        arm_base_f_ext.reserve(6);
        arm_base_f_ext.push_back(wrench_copy[0]); // Fx body
        arm_base_f_ext.push_back(wrench_copy[1]); // Fy body
        arm_base_f_ext.push_back(wrench_copy[2]); // Fz body
        arm_base_f_ext.push_back(wrench_copy[3]); // Tx body
        arm_base_f_ext.push_back(wrench_copy[4]); // Ty body
        arm_base_f_ext.push_back(wrench_copy[5]); // Tz body

        vehicle_simulate_argument = {uv_state, uv_input, vehicle_parameters_new, delta_seconds, arm_base_f_ext};
        vehicle_sim = utils_service.vehicle_dynamics(vehicle_simulate_argument);
        vehicle_next_states = dense_vector(vehicle_sim.at(0), 12);
        if (disturbance_receiver_) disturbance_receiver_->completed(injection, delta_seconds, contact_only);
        // Only actuator effort enters the estimator; external wrench stays plant-only.
        for (int i = 0; i < 6; ++i) estimator_applied_input_(i) = uv_input[i];

        hw_vehicle_struct.current_state_.position_x = vehicle_next_states[0];
        hw_vehicle_struct.current_state_.position_y = vehicle_next_states[1];
        hw_vehicle_struct.current_state_.position_z = vehicle_next_states[2];

        const auto measurement = sample_vehicle_measurements(vehicle_next_states, delta_seconds);
        hw_vehicle_struct.depth_from_pressure2 = measurement[0];

        hw_vehicle_struct.current_state_.setEuler(vehicle_next_states[3],
                                                  vehicle_next_states[4],
                                                  vehicle_next_states[5]);

        hw_vehicle_struct.imu_state.setEuler(measurement[1], -measurement[2], -measurement[3]);

        hw_vehicle_struct.imu_state.roll_unwrap = measurement[1];
        hw_vehicle_struct.imu_state.pitch_unwrap = -measurement[2];
        hw_vehicle_struct.imu_state.yaw_unwrap = -measurement[3];

        hw_vehicle_struct.current_state_.u = vehicle_next_states[6];
        hw_vehicle_struct.current_state_.v = vehicle_next_states[7];
        hw_vehicle_struct.current_state_.w = vehicle_next_states[8];
        hw_vehicle_struct.current_state_.p = vehicle_next_states[9];
        hw_vehicle_struct.current_state_.q = vehicle_next_states[10];
        hw_vehicle_struct.current_state_.r = vehicle_next_states[11];

        // Physical sensor model at the completed state, actual applied effort and
        // actual external loads. Truth is used ONLY to produce simulated readings.
        // The correction above sees noisy readings, nominal parameters and its own d.
        const auto physical = physical_sensor_model_(std::vector<casadi::DM>{
            casadi::DM::vertcat({casadi::DM(vehicle_next_states),casadi::DM(arm_base_f_ext)}),
            casadi::DM::vertcat(uv_input),casadi::DM::vertcat(vehicle_parameters_new),
            imu_offset_,sensor_gravity_});
        const auto physical_acceleration=dense_vector(physical[2],6);
        hw_vehicle_struct.current_state_.du=physical_acceleration[0];
        hw_vehicle_struct.current_state_.dv=physical_acceleration[1];
        hw_vehicle_struct.current_state_.dw=physical_acceleration[2];
        hw_vehicle_struct.current_state_.dp=physical_acceleration[3];
        hw_vehicle_struct.current_state_.dq=physical_acceleration[4];
        hw_vehicle_struct.current_state_.dr=physical_acceleration[5];
        raw_imu_new_.fill(sensor_new_[1]);
        if(sensor_new_[1]) {
            for(int i=0;i<6;++i)
                raw_imu_measurement_[i]=double(physical[0](7+i))+apply_sensor_noise(raw_imu_std_[i]*sensor_normal_(sensor_noise_rng_));
            // FRD sensor model -> ROS FLU sensor interfaces, same timestamp as attitude.
            hw_vehicle_struct.imu_state.angular_vel_x=raw_imu_measurement_[0];
            hw_vehicle_struct.imu_state.angular_vel_y=-raw_imu_measurement_[1];
            hw_vehicle_struct.imu_state.angular_vel_z=-raw_imu_measurement_[2];
            hw_vehicle_struct.imu_state.linear_acceleration_x=raw_imu_measurement_[3];
            hw_vehicle_struct.imu_state.linear_acceleration_y=-raw_imu_measurement_[4];
            hw_vehicle_struct.imu_state.linear_acceleration_z=-raw_imu_measurement_[5];
        }

        hw_vehicle_struct.dvl_state.vx = measurement[4];
        hw_vehicle_struct.dvl_state.vy = measurement[5];
        hw_vehicle_struct.dvl_state.vz = measurement[6];

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.sim_time = time_seconds;
        hw_vehicle_struct.sim_period = delta_seconds;

        sim_step_ += 1.0;
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
