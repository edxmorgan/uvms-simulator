#pragma once

#include <urdf/model.h>
#include <Eigen/Geometry>
#include <array>
#include <stdexcept>
#include <string>

namespace ros2_control_blue_reach_5 {
// Parsed once at initialization. URDF is FLU; vehicle dynamics use FRD.
struct VehicleSensorMounts {
  std::array<double, 9> offsets{}; // IMU, DVL, pressure; body FRD
  Eigen::Matrix3d dvl_to_body = Eigen::Matrix3d::Identity();

  double pressure_depth_offset(const Eigen::Vector3d &rpy) const {
    // Vertical component of R_NED_body * r_body (yaw does not affect depth).
    return -std::sin(rpy[1])*offsets[6] + std::cos(rpy[1])*std::sin(rpy[0])*offsets[7]
        + std::cos(rpy[1])*std::cos(rpy[0])*offsets[8];
  }
  Eigen::Vector3d dvl_lever_velocity(const Eigen::Vector3d &omega) const {
    return omega.cross(Eigen::Vector3d(offsets[3], offsets[4], offsets[5]));
  }

  static VehicleSensorMounts from_urdf(const std::string &xml, const std::string &prefix) {
    urdf::Model model;
    if (!model.initString(xml)) throw std::invalid_argument("Invalid sensor-mount URDF");
    const std::string base = prefix + "base_link";
    const auto mount = [&](const std::string &name) {
      auto link = model.getLink(prefix + name);
      Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
      while (link && link->name != base) {
        const auto joint = link->parent_joint;
        if (!joint || joint->type != urdf::Joint::FIXED)
          throw std::invalid_argument("Sensor must have a fixed URDF chain to " + base + ": " + name);
        const auto &o = joint->parent_to_joint_origin_transform;
        Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
        step.linear() = Eigen::Quaterniond(o.rotation.w, o.rotation.x, o.rotation.y, o.rotation.z).toRotationMatrix();
        step.translation() = Eigen::Vector3d(o.position.x, o.position.y, o.position.z);
        result = step * result;
        link = link->getParent();
      }
      if (!link || !result.matrix().allFinite())
        throw std::invalid_argument("Missing or invalid URDF sensor mount: " + prefix + name);
      return result;
    };
    VehicleSensorMounts out;
    const Eigen::Matrix3d flu_to_frd = Eigen::Vector3d(1., -1., -1.).asDiagonal();
    const std::array<std::string, 3> names{"imu_link", "dvl_link", "pressure_link"};
    for (int i = 0; i < 3; ++i) {
      const auto transform = mount(names[i]);
      const Eigen::Vector3d offset = flu_to_frd * transform.translation();
      for (int j = 0; j < 3; ++j) out.offsets[3*i+j] = offset[j];
      // A50 installation contract: reported axes are local FRD. ROS link is FLU.
      if (i == 1) out.dvl_to_body = flu_to_frd * transform.linear() * flu_to_frd;
    }
    return out;
  }
};
}  // namespace ros2_control_blue_reach_5
