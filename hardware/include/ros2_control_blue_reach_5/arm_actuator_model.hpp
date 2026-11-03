#pragma once

#include <cmath>

namespace ros2_control_blue_reach_5
{
namespace arm_actuator_model
{

// The inverse of current_to_torque on its moving branch. A nonzero requested
// torque must overcome the static-current offset even when its magnitude is
// smaller than static_current / kt; otherwise the round trip produces zero.
template<typename Scalar>
Scalar torque_to_current(Scalar kt, Scalar static_current, Scalar torque)
{
  if (std::abs(torque) <= static_cast<Scalar>(1e-9)) {
    return static_cast<Scalar>(0);
  }
  return kt * torque + std::copysign(static_current, torque);
}

template<typename Scalar>
Scalar current_to_torque(Scalar kt, Scalar static_current, Scalar current)
{
  if (std::abs(current) <= static_current) {
    return static_cast<Scalar>(0);
  }
  return (current - std::copysign(static_current, current)) / kt;
}

}  // namespace arm_actuator_model
}  // namespace ros2_control_blue_reach_5
