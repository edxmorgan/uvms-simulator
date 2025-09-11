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
#include <cmath>
#include <limits>
#include <memory>
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
        utils_service.vehicle_dynamics = utils_service.load_casadi_fun("Vnext", "libUV_xnext.so");
        utils_service.genForces2propThrust = utils_service.load_casadi_fun("F_thrusters", "libF_thrust.so");
        utils_service.thrust2rads = utils_service.load_casadi_fun("thrusts_to_rads", "libTHRUST_RAD.so");

        hw_vehicle_struct.world_frame_id = get_hardware_info().hardware_parameters.at("world_frame_id");
        hw_vehicle_struct.body_frame_id = get_hardware_info().hardware_parameters.at("body_frame_id");
        hw_vehicle_struct.map_frame_id = get_hardware_info().hardware_parameters.at("map_frame_id");
        hw_vehicle_struct.robot_prefix = get_hardware_info().hardware_parameters.at("prefix");

        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************robot prefix: %s", hw_vehicle_struct.robot_prefix.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************frame id: %s", hw_vehicle_struct.world_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************child frame id: %s", hw_vehicle_struct.body_frame_id.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "*************map frame id: %s", hw_vehicle_struct.map_frame_id.c_str());

        // Use the robot_prefix as a seed
        std::size_t seed_val = std::hash<std::string>{}(hw_vehicle_struct.robot_prefix);
        std::mt19937 gen(seed_val + 23);

        // std::uniform_real_distribution<> dis_x(0.0, 0.0);
        // std::uniform_real_distribution<> dis_y(0.0, 0.0);
        // std::uniform_real_distribution<> dis_z(0.0, 0.0);

        // map_position_x = dis_x(gen);
        // map_position_y = dis_y(gen);
        // map_position_z = dis_z(gen);

        map_position_x = 0.0;
        map_position_y = 0.0;
        map_position_z = 0.0;

        map_orientaion_w = 1.0;
        map_orientaion_x = 0.0;
        map_orientaion_y = 0.0;
        map_orientaion_z = 0.0;

        // std::uniform_real_distribution<> robot_dis_x(0.0, -10.0);
        // std::uniform_real_distribution<> robot_dis_y(0.0, 10.0);
        // std::uniform_real_distribution<> robot_dis_z(0.0, 0.0);

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
            // SimVehicleSystemMultiInterfaceHardware has exactly 81 gpio state interfaces
            if (gpio.state_interfaces.size() != 81)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 81 expected.", gpio.name.c_str(),
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
            transform_message.transforms.resize(1);
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
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "configure successful");
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
        const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
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

        // Extract result
        x_est_ = state_est[0];
        P_est_ = state_est[1];
        for (std::size_t i = 0; i < 18; ++i)
        {
            P_diag_[i] = double(P_est_(i, i));
        }

        // // Convert x_est_ to std::vector<double> or just read from DM?
        std::vector<double> x_est_v = x_est_.nonzeros();

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
        delta_seconds = period.seconds();
        time_seconds = time.seconds();

        // IMPORTANT, clear per cycle
        uv_state.clear();
        uv_state.reserve(12);

        uv_input.clear();
        uv_input.reserve(6);

        arm_base_f_ext.clear();
        arm_base_f_ext.reserve(6);

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

        // Define the 6×8 thrust configuration matrix.
        DM thrust_config = DM({{0.707, 0.707, -0.707, -0.707, 0.0, 0.0, 0.0, 0.0},
                               {-0.707, 0.707, -0.707, 0.707, 0.0, 0.0, 0.0, 0.0},
                               {0.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, -1.0},
                               {0.06, -0.06, 0.06, -0.06, -0.218, -0.218, 0.218, 0.218},
                               {0.06, 0.06, -0.06, -0.06, 0.12, -0.12, 0.12, -0.12},
                               {-0.1888, 0.1888, 0.1888, -0.1888, 0.0, 0.0, 0.0, 0.0}});

        std::vector<DM> inputs = {thrust_config, uv_input};
        std::vector<DM> thrust_outputs = utils_service.genForces2propThrust(inputs);
        std::vector<double> thrusts = thrust_outputs.at(0).nonzeros();

        std::vector<DM> thrust2rads_args = {thrusts};
        std::vector<DM> thruster_rads_outputs = utils_service.thrust2rads(thrust2rads_args);
        std::vector<double> thrusts_rads_double = thruster_rads_outputs.at(0).nonzeros();

        for (std::size_t i = 0; i < get_hardware_info().joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position = hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position + thrusts_rads_double[i] * delta_seconds;
        }

        vehicle_parameters_new = {1.15000000e+01, 1.12815000e+02, 1.14800000e+02, 0.00000000e+00,
                                  0.00000000e+00, 2.00000000e-02, 0.00000000e+00, 0.00000000e+00,
                                  0.00000000e+00, 1.60000000e-01, 1.60000000e-01, 1.60000000e-01,
                                  0.00000000e+00, -5.50000000e+00, -1.27000000e+01, -1.45700000e+01,
                                  -1.20000000e-01, -1.20000000e-01, -1.20000000e-01, 0.00000000e+00,
                                  0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -4.03000000e+00,
                                  -6.22000000e+00, -5.18000000e+00, -7.00000000e-02, -7.00000000e-02,
                                  -7.00000000e-02, -1.81800000e+01, -2.16600000e+01, -3.69900000e+01,
                                  -1.55000000e+00, -1.55000000e+00, -1.55000000e+00,
                                  0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                  0.00000000e+00, 0.00000000e+00, 0.00000000e+00};

        arm_base_f_ext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        vehicle_simulate_argument = {uv_state, uv_input, vehicle_parameters_new, delta_seconds, arm_base_f_ext};
        vehicle_sim = utils_service.vehicle_dynamics(vehicle_simulate_argument);
        vehicle_next_states = vehicle_sim.at(0).nonzeros();

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

        publishRealtimePoseTransform();
        return hardware_interface::return_type::OK;
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
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
            "Published static odom transform once during activation.");
    };

    void SimVehicleSystemMultiInterfaceHardware::publishRealtimePoseTransform()
    {
        rclcpp::Time current_time = node_topics_interface_->now();
        if (realtime_transform_publisher_ && realtime_transform_publisher_->trylock())
        {
            auto &transforms = realtime_transform_publisher_->msg_.transforms;
            auto &StateEstimateTransform = transforms.front();
            StateEstimateTransform.header.frame_id = hw_vehicle_struct.map_frame_id;
            StateEstimateTransform.child_frame_id = hw_vehicle_struct.body_frame_id;
            StateEstimateTransform.header.stamp = current_time;
            StateEstimateTransform.transform.translation.x = hw_vehicle_struct.current_state_.position_x;
            StateEstimateTransform.transform.translation.y = -hw_vehicle_struct.current_state_.position_y;
            StateEstimateTransform.transform.translation.z = -hw_vehicle_struct.current_state_.position_z;

            q_orig.setW(hw_vehicle_struct.current_state_.orientation_w);
            q_orig.setX(hw_vehicle_struct.current_state_.orientation_x);
            q_orig.setY(hw_vehicle_struct.current_state_.orientation_y);
            q_orig.setZ(hw_vehicle_struct.current_state_.orientation_z);

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

            // Publish the TF
            realtime_transform_publisher_->unlockAndPublish();
        }
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimVehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
