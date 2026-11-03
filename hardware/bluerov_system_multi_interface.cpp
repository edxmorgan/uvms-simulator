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

#include "ros2_control_blue_reach_5/bluerov_system_multi_interface.hpp"

#include "ros2_control_blue_reach_5/dvldriver.hpp"
#include <angles/angles.h>
#include <chrono>
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
#include <rclcpp/qos.hpp>
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"

using namespace casadi;

namespace ros2_control_blue_reach_5
{
    namespace
    {
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

        template <typename Map>
        bool bool_param(const Map &params, const std::string &name, const bool default_value)
        {
            const auto it = params.find(name);
            if (it == params.cend())
            {
                return default_value;
            }
            return it->second == "true" || it->second == "True" || it->second == "1";
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

        template <typename Map>
        std::string string_param(const Map &params, const std::string &name, const std::string &default_value)
        {
            const auto it = params.find(name);
            return it == params.cend() ? default_value : it->second;
        }

        template <typename Map>
        int int_param(const Map &params, const std::string &name, const int default_value)
        {
            const auto it = params.find(name);
            if (it == params.cend())
            {
                return default_value;
            }
            return std::stoi(it->second);
        }
    } // namespace

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_init(
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
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");
        utils_service.genForces2propThrust = utils_service.load_casadi_fun("F_thrusters", "libF_thrust.so");
        utils_service.from_thrust_to_pwm = utils_service.load_casadi_fun("getNpwm", "libThrust_PWM.so");
        utils_service.uv_Exkalman_update = utils_service.load_casadi_fun("disturbance_ekf_update", "libdisturbanceEKF_next.so");
        estimator_predict_ = utils_service.load_casadi_fun("disturbance_ekf_predict", "libdisturbanceEKF_next.so");
        estimator_correct_ = utils_service.load_casadi_fun("disturbance_ekf_correct", "libdisturbanceEKF_next.so");
        utils_service.pwm_to_thrusts = utils_service.load_casadi_fun("pwm_to_thrusts", "libPWM_THRUST.so");
        estimator_nominal_parameters_ = casadi::DM(default_nominal_vehicle_parameters());
        const auto estimator_model = default_nominal_vehicle_parameters();
        std::copy(estimator_model.begin(), estimator_model.end(), estimator_model_readback_.begin());
        utils_service.pwm2rads = utils_service.load_casadi_fun("pwm_to_rads", "libPWM_RAD.so");
        utils_service.unwrap = utils_service.load_casadi_fun("unwrap", "libAngWrap.so");

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
        use_dvl_ = bool_param(get_hardware_info().hardware_parameters, "use_dvl", false);
        manual_control_ = bool_param(get_hardware_info().hardware_parameters, "manual_control", false);
        // Dropout limits, not measurement-update periods. Pressure telemetry is
        // normally 2 Hz; allow three periods before declaring it unavailable.
        const auto timeout = [&](const char *name, double &value) {
            const auto &params = get_hardware_info().hardware_parameters;
            const auto it = params.find(name);
            if (it != params.end()) value = std::stod(it->second);
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument(std::string(name) + " must be finite and positive");
        };
        try {
            timeout("imu_timeout_s", imu_timeout_s_);
            timeout("pressure_timeout_s", pressure_timeout_s_);
            timeout("dvl_timeout_s", dvl_timeout_s_);
        } catch (const std::exception &error) {
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "%s", error.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        dvl_host_ = string_param(get_hardware_info().hardware_parameters, "dvl_host", dvl_host_);
        dvl_port_ = int_param(get_hardware_info().hardware_parameters, "dvl_port", dvl_port_);

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "robot prefix: %s", hw_vehicle_struct.robot_prefix.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "frame id: %s", hw_vehicle_struct.world_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "child frame id: %s", hw_vehicle_struct.body_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "map frame id: %s", hw_vehicle_struct.map_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "use_pwm: %s", use_pwm_str.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "use_dvl: %s", use_dvl_ ? "true" : "false");
        // map_position_x = 5.0;
        // map_position_y = 5.0;
        // map_position_z = 0.0;

        map_position_x = 0.0;
        map_position_y = 0.0;
        map_position_z = 0.0;

        map_orientaion_w = 1.0;
        map_orientaion_x = 0.0;
        map_orientaion_y = 0.0;
        map_orientaion_z = 0.0;

        blue::dynamics::Vehicle::Pose_vel initial_state{
            0.0, 0.0, 0.0, // map_frame at position: x, y, z
            0.0, 0.0, 0.0,
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
            // Make sure the the joint-level parameters exist
            if (
                joint.parameters.find("param_name") == joint.parameters.cend() ||
                joint.parameters.find("default_param_value") == joint.parameters.cend() ||
                joint.parameters.find("channel") == joint.parameters.cend() ||
                joint.parameters.find("neutral_pwm") == joint.parameters.cend() ||
                joint.parameters.find("direction") == joint.parameters.cend())
            {
                RCLCPP_ERROR( // NOLINT
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Joint %s missing required configurations. Ensure that the `param_name`, `default_param_value`, `neutral_pwm` "
                    "`channel` and `direction` are provided for each joint.",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            };

            rcl_interfaces::msg::Parameter mavros_rc_param;
            mavros_rc_param.name = joint.parameters.at("param_name");
            mavros_rc_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            mavros_rc_param.value.integer_value = std::stoi(joint.parameters.at("default_param_value"));

            int rc_channel = std::stoi(joint.parameters.at("channel"));
            int rc_neutral_pwm = std::stoi(joint.parameters.at("neutral_pwm"));
            int rc_direction = std::stoi(joint.parameters.at("direction"));

            Thruster::State defaultState{};
            hw_vehicle_struct.hw_thrust_structs_.emplace_back(joint.name, mavros_rc_param, rc_channel, rc_neutral_pwm, rc_direction, defaultState);
            // RRBotSystemMultiInterface has exactly 6 joint state interfaces
            if (joint.state_interfaces.size() != 6)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu state interfaces. 6 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };

            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Thruster '%s'has %zu command interfaces. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            };
        };
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "hw_vehicle_struct size: %zu",
                    hw_vehicle_struct.hw_thrust_structs_.size());

        for (const hardware_interface::ComponentInfo &gpio : get_hardware_info().gpios)
        {
            // Make sure the the gpio-level parameters exist
            if (
                gpio.parameters.find("Light1_channel") == gpio.parameters.cend() ||
                gpio.parameters.find("Light2_channel") == gpio.parameters.cend() ||
                gpio.parameters.find("CameraMountPitch_channel") == gpio.parameters.cend())
            {
                RCLCPP_ERROR( // NOLINT
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO %s missing required configurations. Ensure that the `Light1_channel`, `Light2_channel` and `CameraMountPitch_channel` "
                    " are provided for GPIO.",
                    gpio.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            };
            hw_vehicle_struct.light1channel = std::stoi(gpio.parameters.at("Light1_channel"));
            hw_vehicle_struct.light2channel = std::stoi(gpio.parameters.at("Light2_channel"));
            hw_vehicle_struct.cameraMountPitch_channel = std::stoi(gpio.parameters.at("CameraMountPitch_channel"));

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Light1_channel: %d", hw_vehicle_struct.light1channel);
            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Light2_channel: %d", hw_vehicle_struct.light2channel);
            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "CameraMountPitch_channel: %d", hw_vehicle_struct.cameraMountPitch_channel);

            // Navigation/diagnostics and disturbance observer interfaces.
            constexpr std::size_t expected_gpio_state_interfaces = 130;
            if (gpio.state_interfaces.size() != expected_gpio_state_interfaces)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s' has %zu state interfaces. %zu expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size(), expected_gpio_state_interfaces);
                return hardware_interface::CallbackReturn::ERROR;
            }
            // RRBotSystemMultiInterface has exactly 28 gpio command interfaces
            if (gpio.command_interfaces.size() != 28)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu command interfaces. 28 expected.", gpio.name.c_str(),
                    gpio.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
        };

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
        {
            std::scoped_lock lock(imu_mutex_, mavlink_mutex_, dvl_data_mutex_);
            imu_new_msg_ = pressure_new_ = estimator_dvl_new_ = false;
            imu_received_ = pressure_received_ = dvl_received_ = {};
            last_imu_stamp_ = -1;
            last_pressure_stamp_ = 0;
            last_dvl_stamp_ = -1.0;
            first_imu_read = true;
        }
        // setup realtime buffers, ROS publishers ...
        try
        {
            // Initialize node
            node_topics_interface_ = std::make_shared<rclcpp::Node>(system_name + "_topics_interface");
            // Initialize executor and add node
            executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            // executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

            executor_->add_node(node_topics_interface_);

            // Start spinning in a separate thread
            spin_thread_ = std::thread([this]()
                                       { executor_->spin(); });

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Started executor and spinning node_topics_interface: %s", node_topics_interface_->get_name());

            tfBuffer_ = std::make_unique<tf2_ros::Buffer>(node_topics_interface_->get_clock());
            tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

            // Initialize the StaticTransformBroadcaster
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_topics_interface_);

            // override_rc publisher
            override_rc_pub_ = rclcpp::create_publisher<mavros_msgs::msg::OverrideRCIn>(node_topics_interface_,
                                                                                        "mavros/rc/override",
                                                                                        rclcpp::SystemDefaultsQoS());

            rt_override_rc_pub_ = std::make_unique<realtime_tools::RealtimePublisher<mavros_msgs::msg::OverrideRCIn>>(override_rc_pub_);

            rt_override_rc_pub_->lock();
            for (auto &channel : rt_override_rc_pub_->msg_.channels)
            {
                channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
            }
            rt_override_rc_pub_->unlock();

            // tf publisher
            transform_publisher_ = rclcpp::create_publisher<tf>(node_topics_interface_,
                                                                DEFAULT_TRANSFORM_TOPIC,
                                                                rclcpp::SystemDefaultsQoS());
            realtime_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf>>(
                    transform_publisher_);

            auto &transform_message = realtime_transform_publisher_->msg_;
            transform_message.transforms.resize(2);

            // Setup IMU subscription with Reliable QoS for testing
            auto imu_callback =
                [this](const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) -> void
            {
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received IMU message");
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    const int64_t stamp = rclcpp::Time(imu_msg->header.stamp).nanoseconds();
                    if (stamp <= last_imu_stamp_ || imu_msg->orientation_covariance[0] < 0) return;

                    hw_vehicle_struct.imu_state.orientation_w = imu_msg->orientation.w;
                    hw_vehicle_struct.imu_state.orientation_x = imu_msg->orientation.x;
                    hw_vehicle_struct.imu_state.orientation_y = imu_msg->orientation.y;
                    hw_vehicle_struct.imu_state.orientation_z = imu_msg->orientation.z;

                    // Convert the geometry_msgs quaternion to a tf2 quaternion
                    tf2::Quaternion imu_q;
                    tf2::fromMsg(imu_msg->orientation, imu_q);
                    if (!std::isfinite(imu_q.length2()) || imu_q.length2() < 1e-12) return;
                    imu_q.normalize();
                    last_imu_stamp_ = stamp;
                    imu_received_ = std::chrono::steady_clock::now();

                    // Convert quaternion to roll, pitch, yaw
                    tf2::Matrix3x3(imu_q).getRPY(hw_vehicle_struct.imu_state.roll,
                                                 hw_vehicle_struct.imu_state.pitch,
                                                 hw_vehicle_struct.imu_state.yaw);

                    hw_vehicle_struct.imu_state.angular_vel_x = imu_msg->angular_velocity.x;
                    hw_vehicle_struct.imu_state.angular_vel_y = imu_msg->angular_velocity.y;
                    hw_vehicle_struct.imu_state.angular_vel_z = imu_msg->angular_velocity.z;

                    hw_vehicle_struct.imu_state.linear_acceleration_x = imu_msg->linear_acceleration.x;
                    hw_vehicle_struct.imu_state.linear_acceleration_y = imu_msg->linear_acceleration.y;
                    hw_vehicle_struct.imu_state.linear_acceleration_z = imu_msg->linear_acceleration.z;

                    imu_new_msg_ = true;
                }
            };

            auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
            auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

            imu_subscriber_ =
                node_topics_interface_->create_subscription<sensor_msgs::msg::Imu>(
                    "/mavros/imu/data", best_effort_qos, imu_callback);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /mavros/imu/data with Reliable QoS");

            mavlink_sub_ = node_topics_interface_->create_subscription<mavros_msgs::msg::Mavlink>(
                "/uas1/mavlink_source",
                rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
                std::bind(&BlueRovSystemMultiInterfaceHardware::mavlink_data_handler, this, std::placeholders::_1));
            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /uas1/mavlink_source for MAVLink messages.");

            // Subscribe to /alpha/lights
            light_subscriber_ = node_topics_interface_->create_subscription<std_msgs::msg::Float32>(
                "/alpha/lights",
                volatile_qos,
                std::bind(&BlueRovSystemMultiInterfaceHardware::light_callback, this, std::placeholders::_1));

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Subscribed to /alpha/lights with Reliable QoS");

            // Subscribe to /alpha/cameraMountPitch
            camera_mount_pitch_subscriber_ = node_topics_interface_->create_subscription<std_msgs::msg::Float32>(
                "/alpha/cameraMountPitch",
                volatile_qos,
                std::bind(&BlueRovSystemMultiInterfaceHardware::cameraMountPitch_callback, this, std::placeholders::_1));

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Subscribed to /alpha/cameraMountPitch with Reliable QoS");

            if (use_dvl_)
            {
                try
                {
                    // DVL is hardware optional; a missing network sensor must not block vehicle startup.
                    dvl_velocity_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
                        node_topics_interface_, "/dvl/twist", rclcpp::SystemDefaultsQoS());
                    realtime_dvl_velocity_publisher_ =
                        std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(
                            dvl_velocity_publisher_);

                    RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                                "DVL velocity realtime publisher initialized on topic /dvl/twist.");

                    dvl_driver_.subscribe([this](const nlohmann::json &msg)
                                          {
                        // This lambda runs in the DVLDriver poll thread whenever new JSON arrives.
                        dvl_msg = blue::dynamics::DVLMessage::from_json(msg);

                        std::lock_guard<std::mutex> lock(dvl_data_mutex_);

                        switch (dvl_msg.message_type) {
                            case blue::dynamics::DVLMessageType::VELOCITY: {
                                dv_vel = std::get<blue::dynamics::DVLVelocityMessage>(dvl_msg.data);
                                // Reject replayed/out-of-order samples before modifying any
                                // measurement used by the estimator or its validity gate.
                                if (dv_vel.time_of_validity <= last_dvl_stamp_) break;
                                last_dvl_stamp_ = dv_vel.time_of_validity;
                                hw_vehicle_struct.dvl_state.altitude = dv_vel.altitude;
                                hw_vehicle_struct.dvl_state.fom = dv_vel.fom;
                                hw_vehicle_struct.dvl_state.format = dv_vel.format;
                                hw_vehicle_struct.dvl_state.status = dv_vel.status;
                                hw_vehicle_struct.dvl_state.time = dv_vel.time;
                                hw_vehicle_struct.dvl_state.covariance = dv_vel.covariance;
                                hw_vehicle_struct.dvl_state.time_of_transmission = dv_vel.time_of_transmission;
                                hw_vehicle_struct.dvl_state.time_of_validity = dv_vel.time_of_validity;
                                hw_vehicle_struct.dvl_state.transducers = dv_vel.transducers;
                                hw_vehicle_struct.dvl_state.type = dv_vel.type;
                                hw_vehicle_struct.dvl_state.velocity_valid = dv_vel.velocity_valid &&
                                    std::isfinite(dv_vel.vx) && std::isfinite(dv_vel.vy) && std::isfinite(dv_vel.vz);
                                if (hw_vehicle_struct.dvl_state.velocity_valid) {
                                    dvl_received_ = std::chrono::steady_clock::now();
                                    estimator_dvl_new_ = true;
                                }
                                hw_vehicle_struct.dvl_state.vx = dv_vel.vx;
                                hw_vehicle_struct.dvl_state.vy = dv_vel.vy;
                                hw_vehicle_struct.dvl_state.vz = dv_vel.vz;

                                new_dvl_data_available_ = true;
                                break;
                            }
                            case blue::dynamics::DVLMessageType::POSITION_LOCAL: {
                                dvl_pose = std::get<blue::dynamics::DVLPoseMessage>(dvl_msg.data);
                                hw_vehicle_struct.dvl_state.format = dvl_pose.format;
                                hw_vehicle_struct.dvl_state.pitch = dvl_pose.pitch;
                                hw_vehicle_struct.dvl_state.roll = dvl_pose.roll;
                                hw_vehicle_struct.dvl_state.status = dvl_pose.status;
                                hw_vehicle_struct.dvl_state.std_dev = dvl_pose.std_dev;
                                hw_vehicle_struct.dvl_state.ts = dvl_pose.ts;
                                hw_vehicle_struct.dvl_state.type = dvl_pose.type;
                                hw_vehicle_struct.dvl_state.x = dvl_pose.x;
                                hw_vehicle_struct.dvl_state.y = dvl_pose.y;
                                hw_vehicle_struct.dvl_state.yaw = dvl_pose.yaw;
                                hw_vehicle_struct.dvl_state.z = dvl_pose.z;
                                break;
                            }
                            case blue::dynamics::DVLMessageType::UNKNOWN:
                            default:
                                RCLCPP_WARN(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received unknown DVL message type.");
                                break;
                        } });

                    dvl_driver_.start(dvl_host_, dvl_port_);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "DVL disabled after startup failure at %s:%d: %s",
                        dvl_host_.c_str(), dvl_port_, e.what());
                    use_dvl_ = false;
                    dvl_velocity_publisher_.reset();
                    realtime_dvl_velocity_publisher_.reset();
                }
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "DVL disabled.");
            }
        }
        catch (const std::exception &e)
        {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
                e.what());
            return CallbackReturn::ERROR;
        }

        // Initialize state estimate vector (18x1)
        x_est_ = casadi::DM::zeros(18, 1);
        // Initialize state covariance as a 18x18 identity scaled by a small value.
        P_est_ = casadi::DM::eye(18) * 0.001;
        for (int i = 12; i < 18; ++i) P_est_(i, i) = 1.0;
        estimator_initialized_ = false;
        estimator_valid_ = 0.0;
        estimator_step_ = 0.0;
        estimator_applied_input_ = casadi::DM::zeros(6, 1);
        estimated_disturbance_.fill(0.0);
        // right after P_ = diag(Q_vector) and R_ = diag(R_vector);
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
        }

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
        // Continuous covariance intensity; discretized with each prediction period.
        // Provisional hardware tuning, not a real-sensor calibration.
        for (int i = 12; i < 18; ++i) Q_vector(i) = 7e-4;
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

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Initialized P_est_, Q_, and R_ for Kalman filter.");

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "configure successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    BlueRovSystemMultiInterfaceHardware::export_state_interfaces()
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
                get_hardware_info().joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.effort));

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
        const std::array<std::string, 6> axes = {"force.x", "force.y", "force.z",
                                                               "torque.x", "torque.y", "torque.z"};
        for (std::size_t i = 0; i < estimator_model_readback_.size(); ++i)
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimator_nominal_parameter." + std::to_string(i), &estimator_model_readback_[i]);
        for (int i = 0; i < 6; ++i) {
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimated_disturbance." + axes[i], &estimated_disturbance_[i]);
            state_interfaces.emplace_back(get_hardware_info().gpios[0].name,
                "estimated_disturbance.variance." + axes[i], &P_diag_[12+i]);
        }
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name, "estimator_valid", &estimator_valid_);
        state_interfaces.emplace_back(get_hardware_info().gpios[0].name, "estimator_step", &estimator_step_);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BlueRovSystemMultiInterfaceHardware::export_command_interfaces()
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

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::prepare_command_mode_switch(
        const std::vector<std::string> & /*start_interfaces*/,
        const std::vector<std::string> & /*stop_interfaces*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Command Mode Switch successful");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Activating... please wait...");

        publishStaticPoseTransform();

        stop_thrusters();
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "System successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Deactivating... please wait...");

        // Stop the thrusters before switching out of passthrough mode
        stop_thrusters();

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        // A coherent snapshot and single-consumption flags are protected together.
        std::unique_lock<std::mutex> imu_lock(imu_mutex_, std::defer_lock);
        std::unique_lock<std::mutex> pressure_lock(mavlink_mutex_, std::defer_lock);
        std::unique_lock<std::mutex> dvl_lock(dvl_data_mutex_, std::defer_lock);
        std::lock(imu_lock, pressure_lock, dvl_lock);
        const auto now = std::chrono::steady_clock::now();
        const auto fresh = [&](const auto &stamp, double timeout) {
            return stamp != std::chrono::steady_clock::time_point{} &&
                std::chrono::duration<double>(now - stamp).count() <= timeout;
        };
        const bool imu_fresh = fresh(imu_received_, imu_timeout_s_);
        const bool pressure_fresh = fresh(pressure_received_, pressure_timeout_s_);
        const bool dvl_fresh = use_dvl_ && hw_vehicle_struct.dvl_state.velocity_valid &&
            fresh(dvl_received_, dvl_timeout_s_);
        navigation_sensors_valid_ = imu_fresh && pressure_fresh;
        const bool full_observation = navigation_sensors_valid_ && dvl_fresh;
        if (!navigation_sensors_valid_) {
            estimator_valid_ = 0.0;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                *node_topics_interface_->get_clock(), 5000,
                "Sensor dropout: IMU fresh=%d pressure fresh=%d; thrusters held neutral",
                imu_fresh, pressure_fresh);
            // Initialize only from actual observations. After initialization,
            // continue prediction/TF and available corrections, with validity
            // false and commands neutral, rather than repeatedly resetting P.
            if (!estimator_initialized_) return hardware_interface::return_type::OK;
        }
        if (!full_observation) {
            estimator_valid_ = 0.0;
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                *node_topics_interface_->get_clock(), 5000,
                "Full sensor set unavailable (IMU=%d pressure=%d DVL=%d): predicting with available updates; disturbance invalid",
                imu_fresh, pressure_fresh, dvl_fresh);
        }
        if (!std::isfinite(delta_seconds) || delta_seconds <= 0.0 || delta_seconds > 0.5) {
            navigation_sensors_valid_ = false;
            estimator_valid_ = 0.0;
            return hardware_interface::return_type::ERROR;
        }

        if (first_imu_read)
        {
            last_wrapped_roll = hw_vehicle_struct.imu_state.roll;
            unwrap_roll_rt = hw_vehicle_struct.imu_state.roll;

            last_wrapped_pitch = hw_vehicle_struct.imu_state.pitch;
            unwrap_pitch_rt = hw_vehicle_struct.imu_state.pitch;

            last_wrapped_yaw = hw_vehicle_struct.imu_state.yaw;
            unwrap_yaw_rt = hw_vehicle_struct.imu_state.yaw;

            first_imu_read = false;
        };
        std::vector<casadi::DM> roll_wrapping_inputs = {hw_vehicle_struct.imu_state.roll, last_wrapped_roll, unwrap_roll_rt};
        std::vector<casadi::DM> roll_wrap_res = utils_service.unwrap(roll_wrapping_inputs);

        // Extract result
        unwrap_roll_rt = roll_wrap_res[0];
        last_wrapped_roll = roll_wrap_res[1];

        std::vector<casadi::DM> pitch_wrapping_inputs = {hw_vehicle_struct.imu_state.pitch, last_wrapped_pitch, unwrap_pitch_rt};
        std::vector<casadi::DM> pitch_wrap_res = utils_service.unwrap(pitch_wrapping_inputs);

        // Extract result
        unwrap_pitch_rt = pitch_wrap_res[0];
        last_wrapped_pitch = pitch_wrap_res[1];

        std::vector<casadi::DM> yaw_wrapping_inputs = {hw_vehicle_struct.imu_state.yaw, last_wrapped_yaw, unwrap_yaw_rt};
        std::vector<casadi::DM> yaw_wrap_res = utils_service.unwrap(yaw_wrapping_inputs);

        // Extract result
        unwrap_yaw_rt = yaw_wrap_res[0];
        last_wrapped_yaw = yaw_wrap_res[1];

        hw_vehicle_struct.imu_state.roll_unwrap = unwrap_roll_rt.scalar();
        hw_vehicle_struct.imu_state.pitch_unwrap = unwrap_pitch_rt.scalar();
        hw_vehicle_struct.imu_state.yaw_unwrap = unwrap_yaw_rt.scalar();

        // measurements
        casadi::DM y_k = casadi::DM::zeros(7, 1);
        {
            y_k(0) = hw_vehicle_struct.depth_from_pressure2;
            y_k(1) = hw_vehicle_struct.imu_state.roll_unwrap;
            y_k(2) = -hw_vehicle_struct.imu_state.pitch_unwrap;
            y_k(3) = -hw_vehicle_struct.imu_state.yaw_unwrap;
            // Absent velocity entries are placeholders excluded by the mask,
            // never observations of zero velocity.
            if (dvl_fresh) {
                y_k(4) = hw_vehicle_struct.dvl_state.vx;
                y_k(5) = hw_vehicle_struct.dvl_state.vy;
                y_k(6) = hw_vehicle_struct.dvl_state.vz;
            }
        };

        std::vector<casadi::DM> state_est;
        try {
            for (double value : y_k.nonzeros())
                if (!std::isfinite(value)) throw std::runtime_error("nonfinite measurement");
            if (!estimator_initialized_) {
                P_est_ = casadi::DM::eye(18) * 0.001;
                for (int i = 12; i < 18; ++i) P_est_(i,i) = 1.0;
                x_est_(casadi::Slice(9,18)) = casadi::DM::zeros(9,1);
                x_est_(casadi::Slice(2, 6)) = y_k(casadi::Slice(0,4));
                if (dvl_fresh) x_est_(casadi::Slice(6,9)) = y_k(casadi::Slice(4,7));
                else for (int i=6;i<9;++i) P_est_(i,i)=1.0;
                // No truth initialization and no acceleration-as-disturbance states.
                estimator_initialized_ = true;
            } else {
                const auto prediction = estimator_predict_(std::vector<casadi::DM>{
                    x_est_, estimator_applied_input_, estimator_nominal_parameters_, delta_seconds});
                x_est_ = prediction[0];
                P_est_ = casadi::DM::mtimes(std::vector<casadi::DM>{
                    prediction[1], P_est_, prediction[1].T()}) + Q_ * delta_seconds;
                P_est_ = (P_est_ + P_est_.T()) * 0.5;
            }
            casadi::DM mask = casadi::DM::zeros(7,1);
            mask(0) = pressure_new_ && pressure_fresh ? 1.0 : 0.0;
            for (int i=1;i<4;++i) mask(i) = imu_new_msg_ && imu_fresh ? 1.0 : 0.0;
            for (int i=4;i<7;++i) mask(i) = estimator_dvl_new_ && dvl_fresh ? 1.0 : 0.0;
            const auto corrected = estimator_correct_(std::vector<casadi::DM>{
                x_est_, P_est_, y_k, R_, mask, full_observation ? 1.0 : 0.0});
            x_est_ = corrected[0]; P_est_ = corrected[1];
            imu_new_msg_ = pressure_new_ = estimator_dvl_new_ = false;
            {
                // Obtain the shared generated acceleration with a zero-covariance,
                // zero-innovation call, without consuming cached measurements.
                state_est = utils_service.uv_Exkalman_update(std::vector<casadi::DM>{
                    x_est_, casadi::DM::zeros(18,18), estimator_applied_input_,
                    estimator_nominal_parameters_, 0.0, x_est_(casadi::Slice(2,9)),
                    casadi::DM::zeros(18,18), R_});
                state_est[1] = P_est_;
            }
            for (const auto &out : state_est)
                for (double value : out.nonzeros())
                    if (!std::isfinite(value)) throw std::runtime_error("nonfinite EKF output");
            x_est_ = state_est[0];
            P_est_ = state_est[1];
            estimator_valid_ = full_observation ? 1.0 : 0.0;
            estimator_step_ += 1.0;
        } catch (const std::exception &error) {
            navigation_sensors_valid_ = false;
            estimator_valid_ = 0.0;
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                         "Disturbance EKF failed: %s", error.what());
            return hardware_interface::return_type::ERROR;
        }
        imu_lock.unlock();
        pressure_lock.unlock();
        dvl_lock.unlock();
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
        }

        const std::vector<double> x_est_v = dense_vector(x_est_, 18);
        const auto acceleration = dense_vector(state_est[4], 6);
        for (int i = 0; i < 6; ++i) estimated_disturbance_[i] = x_est_v[12+i];

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
        hw_vehicle_struct.current_state_.du = acceleration[0];
        hw_vehicle_struct.current_state_.dv = acceleration[1];
        hw_vehicle_struct.current_state_.dw = acceleration[2];
        hw_vehicle_struct.current_state_.dp = acceleration[3];
        hw_vehicle_struct.current_state_.dq = acceleration[4];
        hw_vehicle_struct.current_state_.dr = acceleration[5];

        // Lock and check if new data is available
        std::lock_guard<std::mutex> lock(dvl_data_mutex_);
        if (new_dvl_data_available_)
        {
            publishDVLVelocity(time);
            new_dvl_data_available_ = false;
        };

        std::vector<DM> pwm_inputs = {{hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm}};

        std::vector<DM> rads_output_dm = utils_service.pwm2rads(pwm_inputs);
        std::vector<double> rads_output = dense_vector(rads_output_dm.at(0), 8);

        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position += rads_output[i] * delta_seconds;
        }

        hw_vehicle_struct.current_state_.position_x = hw_vehicle_struct.estimate_state_.position_x;
        hw_vehicle_struct.current_state_.position_y = hw_vehicle_struct.estimate_state_.position_y;
        hw_vehicle_struct.current_state_.position_z = hw_vehicle_struct.estimate_state_.position_z;

        hw_vehicle_struct.current_state_.setEuler(hw_vehicle_struct.estimate_state_.roll,
                                                  hw_vehicle_struct.estimate_state_.pitch,
                                                  hw_vehicle_struct.estimate_state_.yaw);

        hw_vehicle_struct.current_state_.u = hw_vehicle_struct.estimate_state_.u;
        hw_vehicle_struct.current_state_.v = hw_vehicle_struct.estimate_state_.v;
        hw_vehicle_struct.current_state_.w = hw_vehicle_struct.estimate_state_.w;
        hw_vehicle_struct.current_state_.p = hw_vehicle_struct.estimate_state_.p;
        hw_vehicle_struct.current_state_.q = hw_vehicle_struct.estimate_state_.q;
        hw_vehicle_struct.current_state_.r = hw_vehicle_struct.estimate_state_.r;

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.sim_time = time_seconds;
        hw_vehicle_struct.sim_period = delta_seconds;

        hw_vehicle_struct.hw_thrust_structs_[0].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[1].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[2].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[3].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[4].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[5].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[6].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm;
        hw_vehicle_struct.hw_thrust_structs_[7].current_state_.rc_pwm = hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm;

        // Publish transforms
        publishRealtimePoseTransform();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        (void)time;
        delta_seconds = period.seconds();
        DM user_forces = DM::zeros(6, 1);
        user_forces(0) = hw_vehicle_struct.command_state_.Fx;
        user_forces(1) = hw_vehicle_struct.command_state_.Fy;
        user_forces(2) = hw_vehicle_struct.command_state_.Fz;
        user_forces(3) = hw_vehicle_struct.command_state_.Tx;
        user_forces(4) = hw_vehicle_struct.command_state_.Ty;
        user_forces(5) = hw_vehicle_struct.command_state_.Tz;

        const double surge      = hw_vehicle_struct.current_state_.u;
        const double sway       = hw_vehicle_struct.current_state_.v;
        const double heave      = hw_vehicle_struct.current_state_.w;
        const double roll_rate  = hw_vehicle_struct.current_state_.p;
        const double pitch_rate = hw_vehicle_struct.current_state_.q;
        const double yaw_rate   = hw_vehicle_struct.current_state_.r;

        const double Fx = hw_vehicle_struct.command_state_.Fx;
        const double Fy = hw_vehicle_struct.command_state_.Fy;
        const double Fz = hw_vehicle_struct.command_state_.Fz;
        const double Tx = hw_vehicle_struct.command_state_.Tx;
        const double Ty = hw_vehicle_struct.command_state_.Ty;
        const double Tz = hw_vehicle_struct.command_state_.Tz;

        // Channel-wise absolute mechanical power
        control_power_ =
            std::abs(Fx * surge) +
            std::abs(Fy * sway) +
            std::abs(Fz * heave) +
            std::abs(Tx * roll_rate) +
            std::abs(Ty * pitch_rate) +
            std::abs(Tz * yaw_rate);

        // Accumulate energy
        control_energy_ += control_power_ * delta_seconds;

        RCLCPP_DEBUG(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                     "Got thruster commands: %f %f %f %f %f %f",
                     (double)user_forces(0),
                     (double)user_forces(1),
                     (double)user_forces(2),
                     (double)user_forces(3),
                     (double)user_forces(4),
                     (double)user_forces(5));

        // Define the 6×8 thrust configuration matrix.
        DM thrust_config = DM({{0.707, 0.707, -0.707, -0.707, 0.0, 0.0, 0.0, 0.0},
                               {-0.707, 0.707, -0.707, 0.707, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0},
                               {0.06, -0.06, 0.06, -0.06, -0.218, -0.218, 0.218, 0.218},
                               {0.06, 0.06, -0.06, -0.06, 0.12, -0.12, 0.12, -0.12},
                               {-0.1888, 0.1888, 0.1888, -0.1888, 0.0, 0.0, 0.0, 0.0}});

        std::vector<DM> inputs = {thrust_config, user_forces};
        std::vector<DM> thrust_outputs = utils_service.genForces2propThrust(inputs);
        std::vector<double> thrusts = dense_vector(thrust_outputs.at(0), 8);
        RCLCPP_DEBUG(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                     "Got thruster pwm commands: %f %f %f %f %f %f %f %f",
                     thrusts[0],
                     thrusts[1],
                     thrusts[2],
                     thrusts[3],
                     thrusts[4],
                     thrusts[5],
                     thrusts[6],
                     thrusts[7]);
        std::vector<DM> pwm_input = utils_service.from_thrust_to_pwm(thrust_outputs.at(0));
        std::vector<double> pwm_commands_from_principal_forces_moments = dense_vector(pwm_input.at(0), 8);

        if (!hw_vehicle_struct.use_pwm)
        {
            hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[0];
            hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[1];
            hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[2];
            hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[3];
            hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[4];
            hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[5];
            hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[6];
            hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm = pwm_commands_from_principal_forces_moments[7];
        }

        if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        {
            std::vector<double> sent_model_pwm(8, 1500.0);
            for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
            {
                // Retrieve the thruster struct for brevity
                Thruster thruster = hw_vehicle_struct.hw_thrust_structs_[i];

                // Scale the command_pwm using the rc_direction.
                // For instance, if rc_direction is -1, this inverts the PWM command.

                const double command = (estimator_valid_ == 1.0 || (manual_control_ && navigation_sensors_valid_)) && std::isfinite(thruster.command_state_.command_pwm)
                    ? thruster.command_state_.command_pwm : 1500.0;
                float scaled_diff_pwm = (command - 1500) * thruster.rc_direction;

                float scaled_pwm = std::clamp(1500 + scaled_diff_pwm, 1100.0f, 1900.0f);

                // if (scaled_pwm != 1500) {
                // // Log both the original and scaled PWM values.
                // RCLCPP_DEBUG(
                //     rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                //     "Thruster with direction %d: original pwm = %f, scaled pwm = %f",
                //     thruster.rc_direction, thruster.command_state_.command_pwm, scaled_pwm
                // );
                // }

                // update the corresponding channel
                const int sent_pwm = static_cast<int>(scaled_pwm);
                rt_override_rc_pub_->msg_.channels[thruster.channel - 1] = sent_pwm;
                // Undo configured wiring direction to recover model convention.
                sent_model_pwm[i] = 1500.0 + (sent_pwm - 1500.0) * thruster.rc_direction;
            };

            // Retrieve the light message from the realtime buffer.
            auto light_msg_ptr_ptr = light_msg_buffer_.readFromRT();
            if (light_msg_ptr_ptr && *light_msg_ptr_ptr)
            {
                std_msgs::msg::Float32::SharedPtr light_msg_ptr = *light_msg_ptr_ptr;
                float delta_light_value = light_msg_ptr->data;
                // Update PWM values based on the current command state
                hw_vehicle_struct.light_pwm += delta_light_value;
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                //             "delta_light_value: %f",
                //             delta_light_value);

                // Ensure light_pwm and camera_mountPitch_pwm stays within the range [1100, 1900]
                hw_vehicle_struct.light_pwm = std::clamp(hw_vehicle_struct.light_pwm, 1100.0, 1900.0);
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.light1channel - 1] = static_cast<int>(hw_vehicle_struct.light_pwm);
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.light2channel - 1] = static_cast<int>(hw_vehicle_struct.light_pwm);
            }
            // Retrieve the camera mount pitch message.
            auto cam_msg_ptr_ptr = camera_mount_pitch_msg_buffer_.readFromRT();
            if (cam_msg_ptr_ptr && *cam_msg_ptr_ptr)
            {
                std_msgs::msg::Float32::SharedPtr cam_msg_ptr = *cam_msg_ptr_ptr;
                float current_pitch = cam_msg_ptr->data;
                hw_vehicle_struct.camera_mountPitch_pwm += current_pitch;
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                //             "current_pitch_value: %f",
                //             current_pitch);
                hw_vehicle_struct.camera_mountPitch_pwm = std::clamp(hw_vehicle_struct.camera_mountPitch_pwm, 1100.0, 1900.0);
                updateCameraMountPitchState();
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.cameraMountPitch_channel - 1] = static_cast<int>(hw_vehicle_struct.camera_mountPitch_pwm);
            }

            rt_override_rc_pub_->unlockAndPublish();
            // This is modeled actuator effort, NOT measured thrust or requested force.
            const auto thrust = utils_service.pwm_to_thrusts(std::vector<DM>{DM(sent_model_pwm)});
            estimator_applied_input_ = DM::mtimes(thrust_config, DM(dense_vector(thrust[0], 8)));
        }
        return hardware_interface::return_type::OK;
    }

    void BlueRovSystemMultiInterfaceHardware::updateCameraMountPitchState()
    {
        hw_vehicle_struct.camera_mount_pitch_position =
            camera_mount_pwm_to_pitch(hw_vehicle_struct.camera_mountPitch_pwm);
    }

    void BlueRovSystemMultiInterfaceHardware::publishStaticPoseTransform()
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
        static_map_transform.transform.rotation.x = map_orientaion_x;
        static_map_transform.transform.rotation.y = map_orientaion_y;
        static_map_transform.transform.rotation.z = map_orientaion_z;
        static_map_transform.transform.rotation.w = map_orientaion_w;
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
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
            "Published static odom transform once during activation.");
    };

    void BlueRovSystemMultiInterfaceHardware::publishRealtimePoseTransform()
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
    void BlueRovSystemMultiInterfaceHardware::publishDVLVelocity(const rclcpp::Time &time)
    {
        // Attempt to acquire the lock for real-time publishing
        if (realtime_dvl_velocity_publisher_ && realtime_dvl_velocity_publisher_->trylock())
        {
            // Safely access the message within the realtime publisher
            auto &twist_msg = realtime_dvl_velocity_publisher_->msg_;
            twist_msg.header.stamp = time;
            twist_msg.header.frame_id = hw_vehicle_struct.robot_prefix + "dvl_link";

            // Assign DVL velocity data
            twist_msg.twist.twist.linear.x = hw_vehicle_struct.dvl_state.vx;
            twist_msg.twist.twist.linear.y = hw_vehicle_struct.dvl_state.vy;
            twist_msg.twist.twist.linear.z = hw_vehicle_struct.dvl_state.vz;

            // Convert 3x3 covariance to 6x6 and assign
            twist_msg.twist.covariance = convert3x3To6x6Covariance(hw_vehicle_struct.dvl_state.covariance);

            // Publish the message safely
            realtime_dvl_velocity_publisher_->unlockAndPublish();
            // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Published DVL velocity with covariance.");
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Failed to acquire lock for DVL velocity publishing.");
        }
    }

    //--- Helper function: convertToBytes ---
    // Rebuilds the full MAVLink packet from the ROS message.
    std::vector<uint8_t> BlueRovSystemMultiInterfaceHardware::convertToBytes(
        const mavros_msgs::msg::Mavlink::SharedPtr &msg)
    {
        std::vector<uint8_t> buffer;
        size_t payload_octets = msg->payload64.size();

        // For MAVLink v1.0: header is 6 bytes.
        if (msg->magic == mavros_msgs::msg::Mavlink::MAVLINK_V10)
        {
            buffer.push_back(msg->magic);
            buffer.push_back(msg->len); // payload length (in bytes)
            buffer.push_back(msg->seq);
            buffer.push_back(msg->sysid);
            buffer.push_back(msg->compid);
            buffer.push_back(msg->msgid);
            for (size_t i = 0; i < payload_octets; i++)
            {
                uint64_t val = msg->payload64[i];
                for (int b = 0; b < 8; b++)
                {
                    buffer.push_back(static_cast<uint8_t>((val >> (8 * b)) & 0xFF));
                }
            }
            size_t expectedPacketSize = 6 + msg->len;
            if (buffer.size() > expectedPacketSize)
            {
                buffer.resize(expectedPacketSize);
            }
            buffer.push_back(static_cast<uint8_t>(msg->checksum & 0xFF));
            buffer.push_back(static_cast<uint8_t>((msg->checksum >> 8) & 0xFF));
        }
        // For MAVLink v2.0: header is 10 bytes.
        else
        {
            buffer.push_back(msg->magic);
            buffer.push_back(msg->len);
            buffer.push_back(msg->incompat_flags);
            buffer.push_back(msg->compat_flags);
            buffer.push_back(msg->seq);
            buffer.push_back(msg->sysid);
            buffer.push_back(msg->compid);
            buffer.push_back(msg->msgid & 0xFF);
            buffer.push_back((msg->msgid >> 8) & 0xFF);
            buffer.push_back((msg->msgid >> 16) & 0xFF);
            for (size_t i = 0; i < payload_octets; i++)
            {
                uint64_t val = msg->payload64[i];
                for (int b = 0; b < 8; b++)
                {
                    buffer.push_back(static_cast<uint8_t>((val >> (8 * b)) & 0xFF));
                }
            }
            size_t expectedPacketSize = 10 + msg->len;
            if (buffer.size() > expectedPacketSize)
            {
                buffer.resize(expectedPacketSize);
            }
            buffer.push_back(static_cast<uint8_t>(msg->checksum & 0xFF));
            buffer.push_back(static_cast<uint8_t>((msg->checksum >> 8) & 0xFF));
            for (auto byte : msg->signature)
            {
                buffer.push_back(byte);
            }
        }
        return buffer;
    }

    void BlueRovSystemMultiInterfaceHardware::light_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Write the pointer to the realtime buffer in a thread-safe, nonblocking manner.
        light_msg_buffer_.writeFromNonRT(msg);
    }

    void BlueRovSystemMultiInterfaceHardware::cameraMountPitch_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        camera_mount_pitch_msg_buffer_.writeFromNonRT(msg);
    }

    inline double BlueRovSystemMultiInterfaceHardware::pressureToDepth(double press_abs_hpa, double water_density = 997.0)
    {
        // Convert hPa to Pa
        const double press_abs_pa = press_abs_hpa * 100.0;

        // Standard atmospheric pressure at sea level, in Pascals
        const double press_surface_pa = 101325.0;

        // Compute the overpressure relative to surface
        const double delta_p = press_abs_pa - press_surface_pa;

        // Standard gravitational acceleration (m/s^2)
        const double g = 9.80665;

        // Depth in meters
        const double depth = delta_p / (water_density * g);

        return depth;
    }

    void BlueRovSystemMultiInterfaceHardware::mavlink_data_handler(
        const mavros_msgs::msg::Mavlink::SharedPtr mavros_data)
    {
        // 1. Convert ROS mavlink msg into raw bytes
        std::vector<uint8_t> packet = convertToBytes(mavros_data);

        mavlink_message_t parsed_msg;
        mavlink_status_t status;
        std::memset(&status, 0, sizeof(status));

        bool any_parsed = false;

        // 2. Parse each byte in the buffer
        for (size_t i = 0; i < packet.size(); i++)
        {
            if (mavlink_parse_char(MAVLINK_COMM_0, packet[i], &parsed_msg, &status))
            {
                any_parsed = true;
                RCLCPP_DEBUG(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Parsed MAVLink msg: id=%d, len=%d",
                    parsed_msg.msgid, parsed_msg.len);

                // 3. Handle specific message IDs
                switch (parsed_msg.msgid)
                {
                case MAVLINK_MSG_ID_SCALED_PRESSURE:
                {
                    mavlink_scaled_pressure_t sp{};
                    mavlink_msg_scaled_pressure_decode(&parsed_msg, &sp);

                    RCLCPP_DEBUG(
                        rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "SCALED_PRESSURE: time_boot_ms=%d, press_abs=%.2f, press_diff=%.2f",
                        sp.time_boot_ms, sp.press_abs, sp.press_diff);

                    // currently not in use
                    break;
                }

                case MAVLINK_MSG_ID_SCALED_PRESSURE2:
                {
                    mavlink_scaled_pressure2_t sp2{};
                    mavlink_msg_scaled_pressure2_decode(&parsed_msg, &sp2);

                    // Acquire lock before modifying shared data
                    {
                        std::lock_guard<std::mutex> lock(mavlink_mutex_);
                        if (sp2.time_boot_ms <= last_pressure_stamp_ || !std::isfinite(sp2.press_abs)) break;
                        last_pressure_stamp_ = sp2.time_boot_ms;
                        pressure_received_ = std::chrono::steady_clock::now();
                        pressure_new_ = true;
                        hw_vehicle_struct.depth_from_pressure2 = pressureToDepth(sp2.press_abs, hw_vehicle_struct.water_density);
                    }

                    RCLCPP_DEBUG(
                        rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "SCALED_PRESSURE2: time_boot_ms=%d, press_abs=%.2f, calculated depth=%.2f",
                        sp2.time_boot_ms, sp2.press_abs, hw_vehicle_struct.depth_from_pressure2);
                    break;
                }

                default:
                    // Other messages can be handled or simply ignored
                    break;
                } // end switch
            }
        } // end for

        // 4. If no messages were successfully parsed, optionally log an error
        if (!any_parsed)
        {
            RCLCPP_DEBUG(
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                "No MAVLink messages parsed in this packet!");
        }
    }

    // Convert 3x3 covariance to 6x6 format
    std::array<double, 36> BlueRovSystemMultiInterfaceHardware::convert3x3To6x6Covariance(const blue::dynamics::Covariance &linear_cov)
    {
        std::array<double, 36> full_covariance = {0.0};
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                full_covariance[i * 6 + j] = linear_cov.data[i * 3 + j];
            }
        }
        return full_covariance;
    }

    ros2_control_blue_reach_5::BlueRovSystemMultiInterfaceHardware::~BlueRovSystemMultiInterfaceHardware()
    {
        stop_background_work();
    }

    void BlueRovSystemMultiInterfaceHardware::stop_thrusters()
    {
        navigation_sensors_valid_ = false;
        estimator_valid_ = 0.0;
        estimator_initialized_ = false;
        if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        {
            for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
            {
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.hw_thrust_structs_[i].channel - 1] = hw_vehicle_struct.hw_thrust_structs_[i].neutral_pwm;
            }
            rt_override_rc_pub_->unlockAndPublish();
            estimator_applied_input_ = casadi::DM::zeros(6,1);
        }
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        stop_background_work();
        tfListener_.reset();
        tfBuffer_.reset();
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Cleaned up executor and spin thread.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void BlueRovSystemMultiInterfaceHardware::stop_background_work() noexcept
    {
        try
        {
            if (use_dvl_)
            {
                dvl_driver_.stop();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                "Exception while stopping DVL driver: %s", e.what());
        }

        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        node_topics_interface_.reset();
        executor_.reset();
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::BlueRovSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
