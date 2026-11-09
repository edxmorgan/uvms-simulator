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

#ifndef ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_

#include "ros2_control_blue_reach_5/vehicle_disturbance_noise_adaptation.hpp"
#include "ros2_control_blue_reach_5/disturbance_receiver.hpp"
#include <chrono>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ros2_control_blue_reach_5/visibility_control.h"
#include "ros2_control_blue_reach_5/srv/reset_sim_uvms.hpp"
#include "ros2_control_blue_reach_5/srv/set_sim_dynamics.hpp"
#include "ros2_control_blue_reach_5/srv/get_sim_dynamics.hpp"
#include "ros2_control_blue_reach_5/msg/vehicle_identification.hpp"

#include "ros2_control_blue_reach_5/state.hpp"
#include "ros2_control_blue_reach_5/custom_hardware_interface_type_values.hpp"
#include "ros2_control_blue_reach_5/utils.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float32.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <casadi/casadi.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <mutex>

namespace ros2_control_blue_reach_5
{

    class SimVehicleSystemMultiInterfaceHardware : public hardware_interface::SystemInterface
    {

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SimVehicleSystemMultiInterfaceHardware);

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        ~SimVehicleSystemMultiInterfaceHardware() override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string> &start_interfaces,
            const std::vector<std::string> &stop_interfaces) override;

        // ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        // hardware_interface::return_type perform_command_mode_switch(
        //     const std::vector<std::string> &start_interfaces,
        //     const std::vector<std::string> &stop_interfaces) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ROS2_CONTROL_BLUE_REACH_5_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        friend struct SimEstimatorTestAccess;
        using Identification = ros2_control_blue_reach_5::msg::VehicleIdentification;
        void receive_retained_model(const Identification &message);
        void apply_retained_model();
        void restore_nominal_estimator_model();
        void reset_vehicle_simulation_state();
        void reset_vehicle_simulation_state(
            const ros2_control_blue_reach_5::srv::ResetSimUvms::Request &request);
        void reset_vehicle_estimators();
        void configure_estimator_noise();
        std::array<double, 7> sample_vehicle_measurements(const std::vector<double> &state, double dt = 0.005);
        void seed_vehicle_estimator_from_state(const blue::dynamics::Vehicle::Pose_vel &state);
        void seed_vehicle_sensor_state_from_state(const blue::dynamics::Vehicle::Pose_vel &state);
        void apply_pending_reset_request();
        void stop_ros_interfaces() noexcept;

        // Store the utils function for the robot joints
        casadi_reach_alpha_5::Utils utils_service;

        // Store the state & commands for the robot vehicle
        blue::dynamics::Vehicle hw_vehicle_struct;
        std::string system_name;

        double map_position_x, map_position_y, map_position_z, map_orientation_yaw;
        double map_orientation_w, map_orientation_x, map_orientation_y, map_orientation_z;

        using tf = tf2_msgs::msg::TFMessage;

        tf2::Quaternion q_orig, q_rot, q_new, q_rot_dvl, q_rot_mocap;

        std::shared_ptr<rclcpp::Publisher<tf>> transform_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf>>
            realtime_transform_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf>> physical_transform_publisher_;

        void publishRealtimePoseTransform();
        void publishStaticPoseTransform();
        void updateCameraMountPitchState();
        void cameraMountPitch_callback(const std_msgs::msg::Float32::SharedPtr msg);
        double delta_seconds;
        double time_seconds;

        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread spin_thread_;
        std::shared_ptr<rclcpp::Node> node_topics_interface_;
        std::mutex simulation_state_mutex_;
        std::shared_ptr<DisturbanceReceiver> disturbance_receiver_;
        // Completed integration-step identity; reset changes generation.
        double sim_step_ = 0.0;
        double sim_generation_ = 0.0;
        std::mutex reset_request_mutex_;
        std::condition_variable reset_applied_cv_;
        std::optional<ros2_control_blue_reach_5::srv::ResetSimUvms::Request> pending_reset_request_;
        std::uint64_t reset_request_sequence_{0};
        std::uint64_t reset_applied_sequence_{0};
        std::uint64_t reset_failed_sequence_{0};
        std::string reset_failure_message_;
        rclcpp::Service<ros2_control_blue_reach_5::srv::ResetSimUvms>::SharedPtr reset_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr sensor_noise_service_;
        rclcpp::Service<ros2_control_blue_reach_5::srv::SetSimDynamics>::SharedPtr dynamics_service_;
        rclcpp::Service<ros2_control_blue_reach_5::srv::GetSimDynamics>::SharedPtr dynamics_readback_service_;
        bool commands_held_{false};
        bool dynamics_paused_{false};
        bool use_coupled_dynamics_{false};

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

       // --- Kalman Filter state variables ---
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr evaluation_trace_pub_;
        double evaluation_reset_sequence_{0.};
        casadi::DM x_est_; // [pose NED(6), body velocity(6), body disturbance wrench(6)]
        casadi::DM P_est_; // 18x18 covariance
        casadi::DM Q_;     // Continuous random-walk intensities; integrated along each completed prediction interval
        casadi::DM R_;     // Derived from the simulated measurement variances
        VehicleDisturbanceNoiseBinding noise_adaptation_;
        casadi::Function estimator_covariance_predict_;
        casadi::Function estimator_predict_, estimator_correct_, sensor_correct_, physical_sensor_model_;
        casadi::DM sensor_R_ = casadi::DM::eye(13);
        std::array<double, 6> raw_imu_std_{.001,.001,.001,.01,.01,.01};
        std::array<double, 6> raw_imu_measurement_{}, raw_imu_new_{};
        casadi::DM imu_offset_ = casadi::DM::zeros(3,1);
        double sensor_gravity_ = 9.80665;
        std::array<double, 3> sensor_rates_{10.0, 200.0, 10.0}; // depth, attitude, DVL Hz
        std::array<double, 3> sensor_next_sample_{}, sensor_sample_time_{};
        std::array<double, 7> sensor_measurement_{}, sensor_new_{};
        double sensor_elapsed_{0.0}; // completed dynamics time since reset
        std::array<double, 9> sensor_dvl_factor_{}; // lower factor of body-FRD covariance
        std::array<double, 7> sensor_noise_std_{};
        std::atomic<bool> sensor_noise_requested_{true};
        double sensor_noise_enabled_{1.0}; // Applied on the completed-step sampling boundary.
        double apply_sensor_noise(double value) const { return sensor_noise_enabled_ ? value : 0.0; }
        uint32_t sensor_noise_seed_{0};
        std::mt19937 sensor_noise_rng_{0};
        std::normal_distribution<double> sensor_normal_{0.0, 1.0};
        double P_diag_[18];
        std::array<double,36> disturbance_covariance_{};
        casadi::DM estimator_nominal_parameters_; // Active EKF model, never plant-profile readback
        std::array<double, 33> estimator_model_readback_{};
        rclcpp::Subscription<Identification>::SharedPtr retained_model_subscription_;
        std::optional<Identification> pending_retained_model_;
        std::chrono::steady_clock::time_point retained_model_received_;
        rclcpp::Time estimator_model_reset_stamp_{0, 0, RCL_ROS_TIME};
        double estimator_model_retained_ = 0.0;
        double estimator_model_epoch_ = -1.0;
        double estimator_model_accepted_time_ = -1.0;
        bool estimator_model_net_weight_only_ = false;
        casadi::DM estimator_applied_input_ = casadi::DM::zeros(6, 1);
        double estimator_step_ = 0.0;
        double estimator_valid_ = 0.0;
        std::array<double, 6> estimated_disturbance_{};
        
        std::vector<casadi::DM> uv_state;
        std::vector<casadi::DM> uv_input;
        std::vector<casadi::DM> vehicle_simulate_argument;
        std::vector<casadi::DM> vehicle_sim;
        std::vector<double> vehicle_next_states;
        std::vector<casadi::DM> vehicle_parameters_new;
        std::array<double, 48> thrust_configuration_matrix_{};
        std::vector<double> arm_base_f_ext;

        // Latest external contact wrench in the vehicle body frame, ordered
        // [Fx, Fy, Fz, Tx, Ty, Tz]. Values use the same sign convention as
        // the vehicle dynamics model; the subscriber does not remap signs.
        std::array<double, 6> contact_wrench_body_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // mutex to protect it across callback thread and write()
        std::mutex contact_wrench_mutex_;

        // ROS 2 subscription handle for contact_wrench_body
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr contact_wrench_sub_;

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr camera_mount_pitch_subscriber_;
        realtime_tools::RealtimeBuffer<std_msgs::msg::Float32::SharedPtr> camera_mount_pitch_msg_buffer_;

        double control_energy_{0.0};
        double control_power_{0.0};

    };

} // namespace ros2_control_blue
#endif // ROS2_CONTROL_BLUE_REACH_5__SIM_VEHICLE_SYSTEM_MULTI_INTERFACE_HPP_
