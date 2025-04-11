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

using namespace casadi;

namespace
{
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace ros2_control_blue_reach_5
{
    hardware_interface::CallbackReturn SimVehicleSystemMultiInterfaceHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Access the name from the HardwareInfo
        system_name = info.name;
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "System name: %s", system_name.c_str());

        // Print the CasADi version
        std::string casadi_version = CasadiMeta::version();
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "CasADi computer from vehicle system: %s", casadi_version.c_str());
        RCLCPP_INFO(rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "Testing casadi ready for operations");

        // Use CasADi's "external" to load the compiled functions
        utils_service.usage_cplusplus_checks("test", "libtest.so", "vehicle");

        hw_vehicle_struct.world_frame_id = info_.hardware_parameters["world_frame_id"];
        hw_vehicle_struct.body_frame_id = info_.hardware_parameters["body_frame_id"];
        hw_vehicle_struct.map_frame_id = info_.hardware_parameters["map_frame_id"];
        hw_vehicle_struct.robot_prefix = info_.hardware_parameters["prefix"];

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
            0.0, 0.0, 0.0, // position: x, y, z
            0.0, 0.0, 0.0, // Orientation: r, p, y
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
            Thruster::State defaultState{};
            hw_vehicle_struct.hw_thrust_structs_.emplace_back(joint.name, defaultState);
            // RRBotSystemMultiInterface has exactly 6 joint state interfaces
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

        for (const hardware_interface::ComponentInfo &gpio : info_.gpios)
        {
            // RRBotSystemMultiInterface has exactly 66 gpio state interfaces
            if (gpio.state_interfaces.size() != 66)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
                    "GPIO '%s'has %zu state interfaces. 66 expected.", gpio.name.c_str(),
                    gpio.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }
            // RRBotSystemMultiInterface has exactly 28 gpio command interfaces
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

        RCLCPP_INFO(
            rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"), "configure successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    SimVehicleSystemMultiInterfaceHardware::export_state_interfaces()
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

        // 50-52: State Estimation Position
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[50].name, &hw_vehicle_struct.estimate_state_.position_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[51].name, &hw_vehicle_struct.estimate_state_.position_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[52].name, &hw_vehicle_struct.estimate_state_.position_z));

        // 53-55: State Estimation Orientation (roll, pitch, yaw)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[53].name, &hw_vehicle_struct.estimate_state_.roll));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[54].name, &hw_vehicle_struct.estimate_state_.pitch));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[55].name, &hw_vehicle_struct.estimate_state_.yaw));

        // 56-59: State Estimation Body Orientation (quaternion)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[56].name, &hw_vehicle_struct.estimate_state_.orientation_w));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[57].name, &hw_vehicle_struct.estimate_state_.orientation_x));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[58].name, &hw_vehicle_struct.estimate_state_.orientation_y));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[59].name, &hw_vehicle_struct.estimate_state_.orientation_z));

        // 60-62: State Estimation Linear Velocity (u, v, w)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[60].name, &hw_vehicle_struct.estimate_state_.u));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[61].name, &hw_vehicle_struct.estimate_state_.v));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[62].name, &hw_vehicle_struct.estimate_state_.w));

        // 63-65: State Estimation Angular Velocity (p, q, r)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[63].name, &hw_vehicle_struct.estimate_state_.p));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[64].name, &hw_vehicle_struct.estimate_state_.q));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[0].name, info_.gpios[0].state_interfaces[65].name, &hw_vehicle_struct.estimate_state_.r));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    SimVehicleSystemMultiInterfaceHardware::export_command_interfaces()
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
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // RCLCPP_INFO(
        //     rclcpp::get_logger("SimVehicleSystemMultiInterfaceHardware"),
        //     "Got commands: %f,  %f, %f, %f, %f,  %f",
        //     hw_vehicle_struct.command_state_.Fx,
        //     hw_vehicle_struct.command_state_.Fy,
        //     hw_vehicle_struct.command_state_.Fz,
        //     hw_vehicle_struct.command_state_.Tx,
        //     hw_vehicle_struct.command_state_.Ty,
        //     hw_vehicle_struct.command_state_.Tz);
        delta_seconds = period.seconds();
        time_seconds = time.seconds();
        for (std::size_t i = 0; i < info_.joints.size(); i++)
        {
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_time = time_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.sim_period = delta_seconds;
            hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position = hw_vehicle_struct.hw_thrust_structs_[i].current_state_.position + 60 * delta_seconds;
        }

        hw_vehicle_struct.current_state_.position_x = hw_vehicle_struct.command_state_.position_x;
        hw_vehicle_struct.current_state_.position_y = hw_vehicle_struct.command_state_.position_y;
        hw_vehicle_struct.current_state_.position_z = hw_vehicle_struct.command_state_.position_z;

        hw_vehicle_struct.current_state_.setEuler(hw_vehicle_struct.command_state_.roll,
                                                  hw_vehicle_struct.command_state_.pitch,
                                                  hw_vehicle_struct.command_state_.yaw);
        
        hw_vehicle_struct.imu_state.setEuler(hw_vehicle_struct.command_state_.roll,
                                             hw_vehicle_struct.command_state_.pitch,
                                             -hw_vehicle_struct.command_state_.yaw); // imu yaw is a negative of the simulation response

        hw_vehicle_struct.current_state_.u = hw_vehicle_struct.command_state_.u;
        hw_vehicle_struct.current_state_.v = hw_vehicle_struct.command_state_.v;
        hw_vehicle_struct.current_state_.w = hw_vehicle_struct.command_state_.w;
        hw_vehicle_struct.current_state_.p = hw_vehicle_struct.command_state_.p;
        hw_vehicle_struct.current_state_.q = hw_vehicle_struct.command_state_.q;
        hw_vehicle_struct.current_state_.r = hw_vehicle_struct.command_state_.r;

        hw_vehicle_struct.dvl_state.vx = hw_vehicle_struct.command_state_.u;
        hw_vehicle_struct.dvl_state.vy = hw_vehicle_struct.command_state_.v;
        hw_vehicle_struct.dvl_state.vz = -hw_vehicle_struct.command_state_.w; // dvl velocity in z is also a negative of the simulation response

        hw_vehicle_struct.current_state_.Fx = hw_vehicle_struct.command_state_.Fx;
        hw_vehicle_struct.current_state_.Fy = -hw_vehicle_struct.command_state_.Fy;
        hw_vehicle_struct.current_state_.Fz = -hw_vehicle_struct.command_state_.Fz;
        hw_vehicle_struct.current_state_.Tx = hw_vehicle_struct.command_state_.Tx;
        hw_vehicle_struct.current_state_.Ty = -hw_vehicle_struct.command_state_.Ty;
        hw_vehicle_struct.current_state_.Tz = -hw_vehicle_struct.command_state_.Tz;

        hw_vehicle_struct.sim_time = time_seconds;
        hw_vehicle_struct.sim_period = delta_seconds;

        publishRealtimePoseTransform(time);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SimVehicleSystemMultiInterfaceHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
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

    void SimVehicleSystemMultiInterfaceHardware::publishRealtimePoseTransform(const rclcpp::Time &time)
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
            StateEstimateTransform.transform.rotation.y = -q_orig.y();
            StateEstimateTransform.transform.rotation.z = -q_orig.z();
            StateEstimateTransform.transform.rotation.w = q_orig.w();

            // Publish the TF
            realtime_transform_publisher_->unlockAndPublish();
        }
    }

} // namespace ros2_control_blue_reach_5

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ros2_control_blue_reach_5::SimVehicleSystemMultiInterfaceHardware,
    hardware_interface::SystemInterface)
