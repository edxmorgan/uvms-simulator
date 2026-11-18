#pragma once

#include <cmath>

namespace ros2_control_blue_reach_5
{
namespace arm_actuator_model
{

// Motor effort inferred from measured current, before load/friction losses.
// The configured kt is current units per joint Nm, not Nm per current unit.
// A stationary motor can exert torque: static friction must not erase current
// measurements. This remains an estimate dependent on calibration of kt.
template<typename Scalar>
Scalar current_to_motor_torque(Scalar kt, Scalar current)
{
  return current / kt;
}

// The inverse of current_to_torque on its moving branch. A nonzero requested
// torque includes the static-current offset even below static_current / kt.
// This algebraic inverse requires calibrated offsets before use for hardware
// commands; a round-trip test does not establish physical calibration.
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
