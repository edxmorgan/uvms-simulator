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

TEST(ArmActuatorModel, ObservedStalledWristCommandsExceedDirectionalStaticOffset)
{
  // Hardware trace: SMC requested -0.15..-0.19 Nm, but the old export
  // sent only -7.5..-9.5 mA (inside the configured 20 mA deadband).
  for (double torque : {-0.19, -0.17, -0.15, 0.15, 0.17, 0.19}) {
    const double offset = torque < 0.0 ? 20.0 : 10.0;
    const double current = torque_to_current(50.0, offset, torque);
    EXPECT_GT(std::abs(current), offset);
    EXPECT_NEAR(current_to_torque(50.0, offset, current), torque, 1e-12);
  }
  EXPECT_DOUBLE_EQ(torque_to_current(50.0, 20.0, -0.1), -25.0);
  EXPECT_DOUBLE_EQ(torque_to_current(50.0, 10.0, 0.1), 15.0);
}

TEST(ArmActuatorModel, MeasuredMotorEffortDoesNotDisappearBelowFrictionOffset)
{
  using ros2_control_blue_reach_5::arm_actuator_model::current_to_motor_torque;
  // Measured shoulder currents in the stalled Ours holds, in configured units.
  EXPECT_NEAR(current_to_motor_torque(90.6,-20.513350197),-0.2264166688410596,1e-10);
  EXPECT_NEAR(current_to_motor_torque(90.6,-17.754808783),-0.1959691918653422,1e-10);
  for (double current : {-86.,-43.001,-43.,-42.999,-1.,0.,1.,42.999,43.,43.001,86.}) {
    const double motor=current_to_motor_torque(90.6,current);
    EXPECT_NEAR(90.6*motor,current,1e-12);
    EXPECT_DOUBLE_EQ(motor,-current_to_motor_torque(90.6,-current));
  }
  // Motor torque and the idealized moving-branch net torque are distinct.
  EXPECT_DOUBLE_EQ(current_to_torque(90.6,43.,20.),0.);
  EXPECT_GT(current_to_motor_torque(90.6,20.),0.);
}
