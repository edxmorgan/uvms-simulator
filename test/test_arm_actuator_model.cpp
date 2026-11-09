#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "ros2_control_blue_reach_5/arm_actuator_model.hpp"

using ros2_control_blue_reach_5::arm_actuator_model::current_to_torque;
using ros2_control_blue_reach_5::arm_actuator_model::torque_to_current;

TEST(ArmActuatorModel, UnsaturatedRoundTripIncludesSubThresholdTorques)
{
  constexpr double kt = 90.6;
  constexpr double static_current = 43.0;
  for (double torque : {-0.38, -0.03, 0.0, 0.03, 0.38}) {
    const double current = torque_to_current(kt, static_current, torque);
    EXPECT_NEAR(current_to_torque(kt, static_current, current), torque, 1e-12);
  }
}

TEST(ArmActuatorModel, CurrentLimitBoundsAppliedTorque)
{
  constexpr double kt = 90.6;
  constexpr double static_current = 43.0;
  constexpr double limit = 400.0;
  for (double requested : {-10.0, 10.0}) {
    const double current = std::clamp(torque_to_current(kt, static_current, requested), -limit, limit);
    const double applied = current_to_torque(kt, static_current, current);
    EXPECT_NEAR(applied, std::copysign((limit - static_current) / kt, requested), 1e-12);
  }
  EXPECT_DOUBLE_EQ(torque_to_current(kt, static_current, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(current_to_torque(kt, static_current, 0.0), 0.0);
}

TEST(ArmActuatorModel, DirectionalOffsetsRemainSymmetricInTorque)
{
  constexpr double kt = 50.0;
  EXPECT_NEAR(current_to_torque(kt, 10.0, torque_to_current(kt, 10.0, 0.015)), 0.015, 1e-12);
  EXPECT_NEAR(current_to_torque(kt, 20.0, torque_to_current(kt, 20.0, -0.015)), -0.015, 1e-12);
}
