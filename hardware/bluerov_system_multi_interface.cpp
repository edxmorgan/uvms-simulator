// Copyright 2024, Edward Morgan
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "ros2_control_blue_reach_5/bluerov_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/dvldriver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstring>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <random>
#include <rclcpp/qos.hpp>

using namespace casadi;

namespace
{
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace ros2_control_blue_reach_5
{
    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Access the name from the HardwareInfo
        system_name = info_.name;
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");
        utils_service.genForces2propThrust = utils_service.load_casadi_fun("F_thrusters", "libF_thrust.so");
        utils_service.from_pwm_to_thrust = utils_service.load_casadi_fun("getNpwm", "libThrust_PWM.so");
        utils_service.uv_kalman_update = utils_service.load_casadi_fun("ekf_step", "libUVkalman_update.so");
        utils_service.pwm2rads = utils_service.load_casadi_fun("pwm_to_rads", "libPWM_RAD.so");

        if (info_.hardware_parameters.find("world_frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'world_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.world_frame_id = info_.hardware_parameters["world_frame_id"];

        if (info_.hardware_parameters.find("body_frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'body_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.body_frame_id = info_.hardware_parameters["body_frame_id"];

        if (info_.hardware_parameters.find("map_frame_id") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'map_frame_id' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.map_frame_id = info_.hardware_parameters["map_frame_id"];

        if (info_.hardware_parameters.find("prefix") == info_.hardware_parameters.cend())
        {
            RCLCPP_ERROR( // NOLINT
                rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "The 'prefix' parameter is required.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        hw_vehicle_struct.robot_prefix = info_.hardware_parameters["prefix"];

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************robot prefix: %s", hw_vehicle_struct.robot_prefix.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************frame id: %s", hw_vehicle_struct.world_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************child frame id: %s", hw_vehicle_struct.body_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************map frame id: %s", hw_vehicle_struct.map_frame_id.c_str());

        map_position_x = 5.0;
        map_position_y = 5.0;
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

        hw_vehicle_struct.thrustSizeAllocation(info_.joints.size());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
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
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "*************hw_vehicle_struct size: %zu",
                    hw_vehicle_struct.hw_thrust_structs_.size());

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // RRBotSystemMultiInterface has exactly 50 gpio state interfaces
            if (gpio.state_interfaces.size() != 50)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 50 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
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
            transform_message.transforms.resize(1);

            // Setup IMU subscription with Reliable QoS for testing
            auto imu_callback =
                [this](const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) -> void
            {
                // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "Received IMU message");
                {
                    std::lock_guard<std::mutex> lock(imu_mutex_);

                    hw_vehicle_struct.imu_state.orientation_w = imu_msg->orientation.w;
                    hw_vehicle_struct.imu_state.orientation_x = imu_msg->orientation.x;
                    hw_vehicle_struct.imu_state.orientation_y = imu_msg->orientation.y;
                    hw_vehicle_struct.imu_state.orientation_z = imu_msg->orientation.z;

                    // Convert the geometry_msgs quaternion to a tf2 quaternion
                    tf2::Quaternion imu_q;
                    tf2::fromMsg(imu_msg->orientation, imu_q);

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

            imu_subscriber_ =
                node_topics_interface_->create_subscription<sensor_msgs::msg::Imu>(
                    "/mavros/imu/data", best_effort_qos, imu_callback);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /mavros/imu/data with Reliable QoS");

            mavlink_sub_ = node_topics_interface_->create_subscription<mavros_msgs::msg::Mavlink>(
                "/uas1/mavlink_source",
                best_effort_qos,
                std::bind(&BlueRovSystemMultiInterfaceHardware::mavlink_data_handler, this, std::placeholders::_1));
            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "Subscribed to /uas1/mavlink_source for MAVLink messages.");

            // Initialize the ROS publisher for images.
            image_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Image>(node_topics_interface_, "/alpha/image_raw", rclcpp::SystemDefaultsQoS());
            realtime_image_pub_ =
                std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>(
                    image_pub_);

            // Initialize the realtime dvl publisher
            dvl_velocity_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(node_topics_interface_,
                                                                                                               "/dvl/twist", rclcpp::SystemDefaultsQoS());
            realtime_dvl_velocity_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(
                    dvl_velocity_publisher_);

            RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                        "DVL velocity realtime publisher initialized on topic /dvl/twist.");

            dvl_driver_.subscribe([this](const nlohmann::json &msg)
                                  {
                // This lambda runs in the DVLDriver poll thread whenever new JSON arrives

                // Deserialize into DVLMessage
                dvl_msg = blue::dynamics::DVLMessage::from_json(msg);

                // Lock the mutex to safely update shared data
                std::lock_guard<std::mutex> lock(dvl_data_mutex_);

                switch (dvl_msg.message_type) {
                    case blue::dynamics::DVLMessageType::VELOCITY: {
                        dv_vel = std::get<blue::dynamics::DVLVelocityMessage>(dvl_msg.data);
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
                        hw_vehicle_struct.dvl_state.velocity_valid = dv_vel.velocity_valid;
                        hw_vehicle_struct.dvl_state.vx = dv_vel.vx;
                        hw_vehicle_struct.dvl_state.vy = dv_vel.vy;
                        hw_vehicle_struct.dvl_state.vz = dv_vel.vz;

                        // Set flag to indicate new data is ready
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

            dvl_driver_.start("192.168.2.95", 16171);
        }
        catch (const std::exception &e)
        {
            fprintf(
                stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
                e.what());
            return CallbackReturn::ERROR;
        }

        // Initialize Kalman state as 12x1
        x_est_ = casadi::DM::zeros(12, 1);

        // Initialize covariance P_est_ similarly:
        // this ekf_step uses a diagonal form
        P_est_ = casadi::DM::ones(12, 1) * 0.001;

        // Process noise Q_, it’s diagonal:
        Q_ = casadi::DM::ones(12, 1) * 0.01;

        // Measurement noise R_
        R_ = casadi::DM::zeros(10, 1);
        R_(0) = 0.05; // pz_pressure
        R_(1) = 0.05; // IMU roll
        R_(2) = 0.05; // IMU pitch
        R_(3) = 0.05; // IMU yaw
        R_(4) = 1.0;  // DVL roll
        R_(5) = 1.0;  // DVL pitch
        R_(6) = 1.0;  // DVL yaw
        R_(7) = 0.05; // DVL vx
        R_(8) = 0.05; // DVL vy
        R_(9) = 0.05; // DVL vz

        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Initialized x_est_, P_est_, Q_, and R_ for Kalman filter.");

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"), "configure successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    BlueRovSystemMultiInterfaceHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.velocity));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.acceleration));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.rc_pwm));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.effort));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_TIME, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, custom_hardware_interface::HW_IF_SIM_PERIOD, &hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period));
        }

        // 0-2: Position
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[0].name, &hw_vehicle_struct.current_state_.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[1].name, &hw_vehicle_struct.current_state_.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[2].name, &hw_vehicle_struct.current_state_.position_z));

        // 3-5: Orientation (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[3].name, &hw_vehicle_struct.current_state_.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[4].name, &hw_vehicle_struct.current_state_.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[5].name, &hw_vehicle_struct.current_state_.yaw));

        // 6-9: Body Orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[6].name, &hw_vehicle_struct.current_state_.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[7].name, &hw_vehicle_struct.current_state_.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[8].name, &hw_vehicle_struct.current_state_.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[9].name, &hw_vehicle_struct.current_state_.orientation_z));

        // 10-12: Linear Velocity (u, v, w)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[10].name, &hw_vehicle_struct.current_state_.u));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[11].name, &hw_vehicle_struct.current_state_.v));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[12].name, &hw_vehicle_struct.current_state_.w));

        // 13-15: Angular Velocity (p, q, r)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[13].name, &hw_vehicle_struct.current_state_.p));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[14].name, &hw_vehicle_struct.current_state_.q));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[15].name, &hw_vehicle_struct.current_state_.r));

        // 16-18: Linear Acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[16].name, &hw_vehicle_struct.current_state_.du));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[17].name, &hw_vehicle_struct.current_state_.dv));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[18].name, &hw_vehicle_struct.current_state_.dw));

        // 19-21: Angular Acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[19].name, &hw_vehicle_struct.current_state_.dp));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[20].name, &hw_vehicle_struct.current_state_.dq));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[21].name, &hw_vehicle_struct.current_state_.dr));

        // 22-24: Force
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[22].name, &hw_vehicle_struct.current_state_.Fx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[23].name, &hw_vehicle_struct.current_state_.Fy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[24].name, &hw_vehicle_struct.current_state_.Fz));

        // 25-27: Torque
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[25].name, &hw_vehicle_struct.current_state_.Tx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[26].name, &hw_vehicle_struct.current_state_.Ty));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[27].name, &hw_vehicle_struct.current_state_.Tz));

        // 28-29: Simulation time and period
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[28].name, &hw_vehicle_struct.sim_time));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[29].name, &hw_vehicle_struct.sim_period));

        // 30-42: IMU interfaces
        // 30-32: IMU angles (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[30].name, &hw_vehicle_struct.imu_state.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[31].name, &hw_vehicle_struct.imu_state.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[32].name, &hw_vehicle_struct.imu_state.yaw));

        // 33-36: IMU orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[33].name, &hw_vehicle_struct.imu_state.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[34].name, &hw_vehicle_struct.imu_state.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[35].name, &hw_vehicle_struct.imu_state.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[36].name, &hw_vehicle_struct.imu_state.orientation_z));

        // 37-39: IMU angular velocity
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[37].name, &hw_vehicle_struct.imu_state.angular_vel_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[38].name, &hw_vehicle_struct.imu_state.angular_vel_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[39].name, &hw_vehicle_struct.imu_state.angular_vel_z));

        // 40-42: IMU linear acceleration
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[40].name, &hw_vehicle_struct.imu_state.linear_acceleration_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[41].name, &hw_vehicle_struct.imu_state.linear_acceleration_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[42].name, &hw_vehicle_struct.imu_state.linear_acceleration_z));

        // 43: Depth measurement
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[43].name, &hw_vehicle_struct.depth_from_pressure2));

        // 44-46: DVL gyro (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[44].name, &hw_vehicle_struct.dvl_state.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[45].name, &hw_vehicle_struct.dvl_state.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[46].name, &hw_vehicle_struct.dvl_state.yaw));

        // 47-49: DVL speed (x, y, z)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[47].name, &hw_vehicle_struct.dvl_state.vx));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[48].name, &hw_vehicle_struct.dvl_state.vy));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[49].name, &hw_vehicle_struct.dvl_state.vz));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    BlueRovSystemMultiInterfaceHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[1].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[2].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[3].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[4].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[5].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[6].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[7].name, custom_hardware_interface::HW_IF_PWM, &hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[0].name, &hw_vehicle_struct.command_state_.position_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[1].name, &hw_vehicle_struct.command_state_.position_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[2].name, &hw_vehicle_struct.command_state_.position_z));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[3].name, &hw_vehicle_struct.command_state_.roll));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[4].name, &hw_vehicle_struct.command_state_.pitch));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[5].name, &hw_vehicle_struct.command_state_.yaw));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[6].name, &hw_vehicle_struct.command_state_.orientation_w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[7].name, &hw_vehicle_struct.command_state_.orientation_x));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[8].name, &hw_vehicle_struct.command_state_.orientation_y));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[9].name, &hw_vehicle_struct.command_state_.orientation_z));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[10].name, &hw_vehicle_struct.command_state_.u));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[11].name, &hw_vehicle_struct.command_state_.v));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[12].name, &hw_vehicle_struct.command_state_.w));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[13].name, &hw_vehicle_struct.command_state_.p));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[14].name, &hw_vehicle_struct.command_state_.q));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[15].name, &hw_vehicle_struct.command_state_.r));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[16].name, &hw_vehicle_struct.command_state_.du));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[17].name, &hw_vehicle_struct.command_state_.dv));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[18].name, &hw_vehicle_struct.command_state_.dw));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[19].name, &hw_vehicle_struct.command_state_.dp));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[20].name, &hw_vehicle_struct.command_state_.dq));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[21].name, &hw_vehicle_struct.command_state_.dr));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[22].name, &hw_vehicle_struct.command_state_.Fx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[23].name, &hw_vehicle_struct.command_state_.Fy));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[24].name, &hw_vehicle_struct.command_state_.Fz));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[25].name, &hw_vehicle_struct.command_state_.Tx));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[26].name, &hw_vehicle_struct.command_state_.Ty));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[0].name, info_.gpios[0].command_interfaces[27].name, &hw_vehicle_struct.command_state_.Tz));

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

        // Start the camera stream.
        startCameraStream();
        
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
        // Stop the camera stream.
        stopCameraStream();
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

        // Lock and check if new data is available
        std::lock_guard<std::mutex> lock(dvl_data_mutex_);
        if (new_dvl_data_available_)
        {
            publishDVLVelocity(time);
            new_dvl_data_available_ = false;
        }

        std::vector<DM> pwm_inputs = {{hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm,
                                       hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm}};

        std::vector<DM> rads_output_dm = utils_service.pwm2rads(pwm_inputs);
        std::vector<double> rads_output = rads_output_dm.at(0).nonzeros();

        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position += rads_output[i] * delta_seconds;
        }


        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = -hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = -hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = -hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = -hw_vehicle_struct.command_state_.Tz;

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

        // ----------------------------------------------------------------------------
        //   EKF UPDATE STEP
        //   gather the measurements from the hardware struct
        // ----------------------------------------------------------------------------
        // Prepare the arguments to ekf_step
        double dt_k = delta_seconds;
        double press_depth_k = hw_vehicle_struct.depth_from_pressure2;

        // // IMU roll/pitch/yaw
        // casadi::DM imu_rpy_k = casadi::DM::zeros(3, 1);
        // {
        //     std::lock_guard<std::mutex> lock(imu_mutex_);
        //     imu_rpy_k(0) = hw_vehicle_struct.imu_state.roll;
        //     imu_rpy_k(1) = hw_vehicle_struct.imu_state.pitch;
        //     imu_rpy_k(2) = hw_vehicle_struct.imu_state.yaw;
        // };

        // casadi::DM dvl_rpy_k = casadi::DM::zeros(3, 1);
        // casadi::DM dvl_vk = casadi::DM::zeros(3, 1);
        // {
        //     // DVL RPY
        //     std::lock_guard<std::mutex> lock(dvl_data_mutex_);
        //     dvl_rpy_k(0) = hw_vehicle_struct.dvl_state.roll;
        //     dvl_rpy_k(1) = hw_vehicle_struct.dvl_state.pitch;
        //     dvl_rpy_k(2) = hw_vehicle_struct.dvl_state.yaw;
        //     // DVL velocity
        //     dvl_vk(0) = hw_vehicle_struct.dvl_state.vx;
        //     dvl_vk(1) = hw_vehicle_struct.dvl_state.vy;
        //     dvl_vk(2) = hw_vehicle_struct.dvl_state.vz;
        // }

        // // Wrap them into a vector for the ekf_step function
        // std::vector<casadi::DM> ekf_inputs = {
        //     x_est_,
        //     P_est_,
        //     casadi::DM(dt_k),
        //     casadi::DM(press_depth_k),
        //     imu_rpy_k,
        //     dvl_rpy_k,
        //     dvl_vk,
        //     Q_,
        //     R_};

        // // Call your CasADi function
        // std::vector<casadi::DM> state_est = utils_service.uv_kalman_update(ekf_inputs);

        // // Extract result
        // x_est_ = state_est[0];
        // P_est_ = state_est[1];

        // // Convert x_est_ to std::vector<double> or just read from DM?
        // std::vector<double> x_est_v = x_est_.nonzeros();

        // // Suppose the state is:
        // //    x_est_ = [px, py, pz, roll, pitch, yaw, u, v, w, p, q, r]
        // hw_vehicle_struct.current_state_.position_x = x_est_v[0];
        // hw_vehicle_struct.current_state_.position_y = x_est_v[1];
        // hw_vehicle_struct.current_state_.position_z = x_est_v[2];

        // // For orientation, we store roll/pitch/yaw and also convert to quaternion
        // hw_vehicle_struct.current_state_.roll = x_est_v[3];
        // hw_vehicle_struct.current_state_.pitch = x_est_v[4];
        // hw_vehicle_struct.current_state_.yaw = x_est_v[5];

        // hw_vehicle_struct.current_state_.setEuler(x_est_v[3], x_est_v[4], x_est_v[5]);

        // // The next states are linear (u,v,w) and angular (p,q,r) velocities
        // hw_vehicle_struct.current_state_.u = x_est_v[6];
        // hw_vehicle_struct.current_state_.v = x_est_v[7];
        // hw_vehicle_struct.current_state_.w = x_est_v[8];
        // hw_vehicle_struct.current_state_.p = x_est_v[9];
        // hw_vehicle_struct.current_state_.q = x_est_v[10];
        // hw_vehicle_struct.current_state_.r = x_est_v[11];

        // Publish transforms
        publishRealtimePoseTransform(time);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BlueRovSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        DM user_forces = DM::zeros(6, 1);
        user_forces(0) = hw_vehicle_struct.command_state_.Fx;
        user_forces(1) = hw_vehicle_struct.command_state_.Fy;
        user_forces(2) = hw_vehicle_struct.command_state_.Fz;
        user_forces(3) = hw_vehicle_struct.command_state_.Tx;
        user_forces(4) = hw_vehicle_struct.command_state_.Ty;
        user_forces(5) = hw_vehicle_struct.command_state_.Tz;

        // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
        //             "Got thruster commands: %f %f %f %f %f %f",
        //             (double)user_forces(0),
        //             (double)user_forces(1),
        //             (double)user_forces(2),
        //             (double)user_forces(3),
        //             (double)user_forces(4),
        //             (double)user_forces(5));

        // Define the 6×8 thrust configuration matrix.
        DM thrust_config = DM({{0.707, 0.707, -0.707, -0.707, 0.0, 0.0, 0.0, 0.0},
                               {-0.707, 0.707, -0.707, 0.707, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0},
                               {0.06, -0.06, 0.06, -0.06, -0.218, -0.218, 0.218, 0.218},
                               {0.06, 0.06, -0.06, -0.06, 0.12, -0.12, 0.12, -0.12},
                               {-0.1888, 0.1888, 0.1888, -0.1888, 0.0, 0.0, 0.0, 0.0}});

        std::vector<DM> inputs = {thrust_config, user_forces};
        std::vector<DM> thrust_outputs = utils_service.genForces2propThrust(inputs);
        std::vector<double> thrusts = thrust_outputs.at(0).nonzeros();
        // RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
        //                             "Got thruster pwm commands: %f %f %f %f %f %f %f %f",
        //                             thrusts[0],
        //                             thrusts[1],
        //                             thrusts[2],
        //                             thrusts[3],
        //                             thrusts[4],
        //                             thrusts[5],
        //                             thrusts[6],
        //                             thrusts[7]);
        std::vector<DM> pwm_input = utils_service.from_pwm_to_thrust(thrust_outputs.at(0));
        std::vector<double> pwm_commands = pwm_input.at(0).nonzeros();

        hw_vehicle_struct.hw_thrust_structs_[0].command_state_.command_pwm = pwm_commands[0];
        hw_vehicle_struct.hw_thrust_structs_[1].command_state_.command_pwm = pwm_commands[1];
        hw_vehicle_struct.hw_thrust_structs_[2].command_state_.command_pwm = pwm_commands[2];
        hw_vehicle_struct.hw_thrust_structs_[3].command_state_.command_pwm = pwm_commands[3];
        hw_vehicle_struct.hw_thrust_structs_[4].command_state_.command_pwm = pwm_commands[4];
        hw_vehicle_struct.hw_thrust_structs_[5].command_state_.command_pwm = pwm_commands[5];
        hw_vehicle_struct.hw_thrust_structs_[6].command_state_.command_pwm = pwm_commands[6];
        hw_vehicle_struct.hw_thrust_structs_[7].command_state_.command_pwm = pwm_commands[7];

        if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        {
            for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
            {
                // Retrieve the thruster struct for brevity
                Thruster thruster = hw_vehicle_struct.hw_thrust_structs_[i];

                // Scale the command_pwm using the rc_direction.
                // For instance, if rc_direction is -1, this inverts the PWM command.

                float scaled_diff_pwm = (thruster.command_state_.command_pwm - 1500) * thruster.rc_direction;

                float scaled_pwm = 1500 + scaled_diff_pwm;

                // if (scaled_pwm != 1500) {
                // // Log both the original and scaled PWM values.
                // RCLCPP_INFO(
                //     rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                //     "Thruster with direction %d: original pwm = %f, scaled pwm = %f",
                //     thruster.rc_direction, thruster.command_state_.command_pwm, scaled_pwm
                // );
                // }

                // If needed, update the corresponding channel (assuming channel indexing starts at 1)
                rt_override_rc_pub_->msg_.channels[thruster.channel - 1] = static_cast<int>(scaled_pwm);
            }

            rt_override_rc_pub_->unlockAndPublish();
        }
        return hardware_interface::return_type::OK;
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

        RCLCPP_INFO(
            rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
            "Published static odom transform once during activation.");
    };

    void BlueRovSystemMultiInterfaceHardware::publishRealtimePoseTransform(const rclcpp::Time &time)
    {
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            auto &transforms = realtime_transform_publisher_->msg_.transforms;
            auto &StateEstimateTransform = transforms.front();
            StateEstimateTransform.header.frame_id = hw_vehicle_struct.map_frame_id;
            StateEstimateTransform.child_frame_id = hw_vehicle_struct.body_frame_id;
            StateEstimateTransform.header.stamp = time;
            StateEstimateTransform.transform.translation.x = hw_vehicle_struct.current_state_.position_x;
            StateEstimateTransform.transform.translation.y = -hw_vehicle_struct.current_state_.position_y;
            StateEstimateTransform.transform.translation.z = -hw_vehicle_struct.current_state_.position_z;

            q_orig.setW(hw_vehicle_struct.current_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct.current_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct.current_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct.current_state_.orientation_z);

            q_orig.normalize();

            StateEstimateTransform.transform.rotation.x = q_orig.x();
            StateEstimateTransform.transform.rotation.y = q_orig.y();
            StateEstimateTransform.transform.rotation.z = q_orig.z();
            StateEstimateTransform.transform.rotation.w = q_orig.w();

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

    void BlueRovSystemMultiInterfaceHardware::startCameraStream()
    {
        // Ensure GStreamer is initialized exactly once.
        static bool gst_initialized = false;
        if (!gst_initialized)
        {
            gst_init(nullptr, nullptr);
            gst_initialized = true;
        }

        // Define the GStreamer pipeline string.
        // Note: "h264parse" has been removed since it is unavailable.
        std::string pipeline_str =
            "udpsrc port=5600 ! application/x-rtp, payload=96 "
            "! rtph264depay ! avdec_h264 "
            "! videoconvert ! video/x-raw,format=(string)BGR "
            "! appsink name=camera_sink emit-signals=true sync=false max-buffers=2 drop=true";

        // Create and start the pipeline.
        gst_pipeline_ = gst_parse_launch(pipeline_str.c_str(), nullptr);
        if (!gst_pipeline_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                         "Failed to create GStreamer pipeline for camera stream");
            return;
        }
        gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

        // Retrieve the appsink element by name.
        GstElement *appsink_elem = gst_bin_get_by_name(GST_BIN(gst_pipeline_), "camera_sink");
        if (!appsink_elem)
        {
            RCLCPP_ERROR(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                         "Failed to get appsink element from camera pipeline");
            return;
        }
        gst_appsink_ = GST_APP_SINK(appsink_elem);

        // Start the camera thread.
        camera_thread_running_ = true;
        camera_thread_ = std::thread(&BlueRovSystemMultiInterfaceHardware::cameraLoop, this);
    }

    void BlueRovSystemMultiInterfaceHardware::cameraLoop()
    {
        while (camera_thread_running_)
        {
            // Pull a sample from the appsink (blocking call).
            GstSample *sample = gst_app_sink_pull_sample(gst_appsink_);
            if (!sample)
            {
                RCLCPP_WARN(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                            "No sample received from appsink");
                continue;
            }

            // Get the buffer and its capabilities.
            GstBuffer *buffer = gst_sample_get_buffer(sample);
            GstCaps *caps = gst_sample_get_caps(sample);
            if (!caps)
            {
                gst_sample_unref(sample);
                continue;
            }

            // Retrieve width and height from the caps.
            GstStructure *s = gst_caps_get_structure(caps, 0);
            int width = 0, height = 0;
            if (!gst_structure_get_int(s, "width", &width) || !gst_structure_get_int(s, "height", &height))
            {
                gst_sample_unref(sample);
                continue;
            }

            // Map the buffer to read the image data.
            GstMapInfo map;
            if (!gst_buffer_map(buffer, &map, GST_MAP_READ))
            {
                gst_sample_unref(sample);
                continue;
            }

            // Create an OpenCV Mat from the buffer data (BGR format).
            cv::Mat frame(height, width, CV_8UC3, reinterpret_cast<char *>(map.data));
            cv::Mat frame_copy = frame.clone(); // Clone the frame as the underlying data will be unmapped.

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);

            // Convert the cv::Mat to a ROS 2 sensor_msgs::Image using cv_bridge.
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp = node_topics_interface_->now(); // Use your node's clock
            cv_image.header.frame_id = "camera_frame";
            cv_image.encoding = "bgr8";
            cv_image.image = frame_copy;

            auto image_msg = cv_image.toImageMsg();
            if (realtime_image_pub_ && realtime_image_pub_->trylock())
            {
                realtime_image_pub_->msg_ = *image_msg; // Copy or assign your message.
                realtime_image_pub_->unlockAndPublish();
            }
        }
    }

    void BlueRovSystemMultiInterfaceHardware::stopCameraStream()
    {
        camera_thread_running_ = false;
        if (camera_thread_.joinable())
        {
            camera_thread_.join();
        }
        if (gst_pipeline_)
        {
            gst_element_set_state(gst_pipeline_, GST_STATE_NULL);
            gst_object_unref(gst_pipeline_);
            gst_pipeline_ = nullptr;
            gst_appsink_ = nullptr;
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
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Executor stopped and spin thread joined.");
    }

    void BlueRovSystemMultiInterfaceHardware::stop_thrusters()
    {
        if (rt_override_rc_pub_ && rt_override_rc_pub_->trylock())
        {
            for (size_t i = 0; i < hw_vehicle_struct.hw_thrust_structs_.size(); ++i)
            {
                rt_override_rc_pub_->msg_.channels[hw_vehicle_struct.hw_thrust_structs_[i].channel - 1] = hw_vehicle_struct.hw_thrust_structs_[i].neutral_pwm;
            }
            rt_override_rc_pub_->unlockAndPublish();
        }
    }

    hardware_interface::CallbackReturn BlueRovSystemMultiInterfaceHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("BlueRovSystemMultiInterfaceHardware"),
                    "Cleaned up executor and spin thread.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::BlueRovSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
