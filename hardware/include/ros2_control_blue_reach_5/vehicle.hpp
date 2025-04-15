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


// #pragma once
#ifndef ROS2_CONTROL_BLUE_REACH_5__VEHICLE_HPP_
#define ROS2_CONTROL_BLUE_REACH_5__VEHICLE_HPP_

#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ros2_control_blue_reach_5/thruster.hpp"
#include "ros2_control_blue_reach_5/eigen.hpp"
#include <ros2_control_blue_reach_5/json.hpp>
#include <variant> // Add this line
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace blue::dynamics
{

  // 1. Define Transducer Struct
  struct Transducer
  {
    bool beam_valid;
    double distance;
    int id;
    double nsd;
    double rssi;
    double velocity;

    // Method to deserialize from JSON
    static Transducer from_json(const nlohmann::json &j)
    {
      Transducer t;
      t.beam_valid = j.at("beam_valid").get<bool>();
      t.distance = j.at("distance").get<double>();
      t.id = j.at("id").get<int>();
      t.nsd = j.at("nsd").get<double>();
      t.rssi = j.at("rssi").get<double>();
      t.velocity = j.at("velocity").get<double>();
      return t;
    }
  };

  // 2. Define Covariance Struct
  struct Covariance
  {
    std::array<double, 9> data;

    // Method to deserialize from JSON
    static Covariance from_json(const nlohmann::json &j)
    {
      Covariance cov;
      if (j.size() != 3)
      {
        throw std::invalid_argument("Covariance must have 3 rows.");
      }
      for (size_t i = 0; i < 3; ++i)
      {
        if (!j[i].is_array() || j[i].size() != 3)
        {
          throw std::invalid_argument("Each covariance row must have 3 elements.");
        }
        for (size_t j_col = 0; j_col < 3; ++j_col)
        {
          cov.data[i * 3 + j_col] = j[i][j_col].get<double>();
        }
      }
      return cov;
    }
  };

  // 3. Define DVLVelocityMessage Struct
  struct DVLVelocityMessage
  {
    double altitude;
    Covariance covariance;
    double fom;
    std::string format;
    int status;
    double time;
    long long time_of_transmission;
    long long time_of_validity;
    std::vector<Transducer> transducers;
    std::string type;
    bool velocity_valid;
    double vx;
    double vy;
    double vz;

    // Method to deserialize from JSON
    static DVLVelocityMessage from_json(const nlohmann::json &j)
    {
      DVLVelocityMessage msg;
      msg.altitude = j.at("altitude").get<double>();
      msg.covariance = Covariance::from_json(j.at("covariance"));
      msg.fom = j.at("fom").get<double>();
      msg.format = j.at("format").get<std::string>();
      msg.status = j.at("status").get<int>();
      msg.time = j.at("time").get<double>();
      msg.time_of_transmission = j.at("time_of_transmission").get<long long>();
      msg.time_of_validity = j.at("time_of_validity").get<long long>();

      // Deserialize transducers
      for (const auto &trans_j : j.at("transducers"))
      {
        msg.transducers.emplace_back(Transducer::from_json(trans_j));
      }

      msg.type = j.at("type").get<std::string>();
      msg.velocity_valid = j.at("velocity_valid").get<bool>();
      msg.vx = j.at("vx").get<double>();
      msg.vy = j.at("vy").get<double>();
      msg.vz = j.at("vz").get<double>();
      return msg;
    }
  };

  // 4. Define DVLPoseMessage Struct
  struct DVLPoseMessage
  {
    std::string format;
    double pitch;
    double roll;
    int status;
    double std_dev; // Renamed to avoid conflict with std::std
    double ts;
    std::string type;
    double x;
    double y;
    double yaw;
    double z;

    // Method to deserialize from JSON
    static DVLPoseMessage from_json(const nlohmann::json &j)
    {
      DVLPoseMessage msg;
      msg.format = j.at("format").get<std::string>();
      msg.pitch = j.at("pitch").get<double>();
      msg.roll = j.at("roll").get<double>();
      msg.status = j.at("status").get<int>();
      msg.std_dev = j.at("std").get<double>();
      msg.ts = j.at("ts").get<double>();
      msg.type = j.at("type").get<std::string>();
      msg.x = j.at("x").get<double>();
      msg.y = j.at("y").get<double>();
      msg.yaw = j.at("yaw").get<double>();
      msg.z = j.at("z").get<double>();
      return msg;
    }
  };

  // 5. Define DVLMessageType Enumeration
  enum class DVLMessageType
  {
    VELOCITY,
    POSITION_LOCAL,
    UNKNOWN
  };

  // 6. Define DVLMessage Struct Using std::variant
  struct DVLMessage
  {
    DVLMessageType message_type;

    // Variant to hold either Velocity or Pose message
    std::variant<DVLVelocityMessage, DVLPoseMessage> data;

    // Method to deserialize from JSON
    static DVLMessage from_json(const nlohmann::json &j)
    {
      DVLMessage msg;
      if (!j.contains("type") || !j["type"].is_string())
      {
        msg.message_type = DVLMessageType::UNKNOWN;
        return msg;
      }

      std::string type_str = j["type"].get<std::string>();
      if (type_str == "velocity")
      {
        msg.message_type = DVLMessageType::VELOCITY;
        msg.data = DVLVelocityMessage::from_json(j);
      }
      else if (type_str == "position_local")
      {
        msg.message_type = DVLMessageType::POSITION_LOCAL;
        msg.data = DVLPoseMessage::from_json(j);
      }
      else
      {
        msg.message_type = DVLMessageType::UNKNOWN;
      }
      return msg;
    }
  };

  class Vehicle
  {

  public:
    std::string name;           // Name of the device or component
    uint8_t device_id;          // Unique identifier for the device
    std::string world_frame_id; // origin frame
    std::string body_frame_id;  // body frame
    std::string map_frame_id;   // map frame
    std::string robot_prefix;   // robot prefix
    int light1channel;
    int light2channel;
    int cameraMountPitch_channel;

    double light_pwm = 1100;
    double camera_mountPitch_pwm = 1500;

    double sim_time = 0;
    double sim_period = 0;

    struct Pose_vel
    {
      double position_x, position_y, position_z;
      double roll, pitch, yaw;
      double orientation_w, orientation_x, orientation_y, orientation_z;
      double u, v, w, p, q, r;
      double du, dv, dw, dp, dq, dr;
      double Fx = 0;
      double Fy = 0;
      double Fz = 0;
      double Tx = 0;
      double Ty = 0;
      double Tz = 0;

      void updateQuaternion()
      {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        q.normalize();

        // Calculate quaternion components
        orientation_w = q.getW();
        orientation_x = q.getX();
        orientation_y = q.getY();
        orientation_z = q.getZ();
      }

      // Method to set Euler angles and update quaternion
      void setEuler(double new_roll, double new_pitch, double new_yaw)
      {
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
        updateQuaternion();
      }

      void updateEuler()
      {
        tf2::Quaternion q;
        q.setW(orientation_w);
        q.setX(orientation_x);
        q.setY(orientation_x);
        q.setZ(orientation_z);

        q.normalize();

        // Calculate euler components
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      }
      // Method to set quaternion angles and update euler
      void setQuaternion(double new_W, double new_X, double new_Y, double new_Z)
      {
        orientation_w = new_W;
        orientation_x = new_X;
        orientation_y = new_Y;
        orientation_z = new_Z;
        updateEuler();
      }
    };

    struct dvl_info
    {
      // DVL Data
      double altitude{0.0};
      Covariance covariance; // Flattened 3x3 covariance matrix
      double fom{0.0};
      std::string format{"unknown"};
      int status{0};
      double time{0.0};
      long long time_of_transmission{0};
      long long time_of_validity{0};
      std::vector<Transducer> transducers;
      std::string type{"unknown"};
      double velocity_valid{0.0}; // 1.0 for true, 0.0 for false
      double vx{0.0};
      double vy{0.0};
      double vz{0.0};

      // For "position_local" type
      double pitch{0.0};
      double roll{0.0};
      double std_dev{0.0};
      double ts{0.0};
      double x{0.0};
      double y{0.0};
      double yaw{0.0};
      double z{0.0};
    };

    struct imu_info
    {
      double roll{0.0};
      double roll_unwrap{0.0};
      double pitch{0.0};
      double pitch_unwrap{0.0};
      double yaw{0.0};
      double yaw_unwrap{0.0};
      double orientation_w{0.0};
      double orientation_x{0.0};
      double orientation_y{0.0};
      double orientation_z{0.0};
      double angular_vel_x{0.0};
      double angular_vel_y{0.0};
      double angular_vel_z{0.0};
      double linear_acceleration_x{0.0};
      double linear_acceleration_y{0.0};
      double linear_acceleration_z{0.0};

      void updateQuaternion()
      {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        q.normalize();

        // Calculate quaternion components
        orientation_w = q.getW();
        orientation_x = q.getX();
        orientation_y = q.getY();
        orientation_z = q.getZ();
      }

      // Method to set Euler angles and update quaternion
      void setEuler(double new_roll, double new_pitch, double new_yaw)
      {
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
        updateQuaternion();
      }

      void updateEuler()
      {
        tf2::Quaternion q;
        q.setW(orientation_w);
        q.setX(orientation_x);
        q.setY(orientation_x);
        q.setZ(orientation_z);

        q.normalize();

        // Calculate euler components
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      }
      // Method to set quaternion angles and update euler
      void setQuaternion(double new_W, double new_X, double new_Y, double new_Z)
      {
        orientation_w = new_W;
        orientation_x = new_X;
        orientation_y = new_Y;
        orientation_z = new_Z;
        updateEuler();
      }
    };

    double depth_from_pressure2{0.0};
    double water_density{1000.0}; // typical freshwater density

    Pose_vel default_state_, command_state_, current_state_, async_state_, estimate_state_;
    dvl_info dvl_state;
    imu_info imu_state;
    std::vector<Thruster> hw_thrust_structs_; // Store the state & commands for the uv thruster

    Vehicle() = default;

    void set_vehicle_name(const std::string &vehicle_name, const Pose_vel &default_state);

    void thrustSizeAllocation(const double &joint_size); // Attempt to preallocate enough memory for specified number of elements

  private:
    // Example: Vector of Transducers
    std::vector<Transducer> transducers;
  };
} // namespace blue::dynamics
#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_
