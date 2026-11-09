#include <gtest/gtest.h>

#include <vector>

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"

using ros2_control_blue_reach_5::BatchUvmsCore;

TEST(BatchArmActuation, SubStaticThresholdCommandMovesReleasedJoint)
{
  BatchUvmsCore core;
  core.configure(1);
  std::vector<double> initial(BatchUvmsCore::kObservationDim, 0.0);
  initial[BatchUvmsCore::kVehicleStateDim + 0] = 3.1;
  initial[BatchUvmsCore::kVehicleStateDim + 1] = 0.7;
  initial[BatchUvmsCore::kVehicleStateDim + 2] = 0.4;
  initial[BatchUvmsCore::kVehicleStateDim + 3] = 2.1;
  initial[BatchUvmsCore::kVehicleStateDim + 4] = 0.006;
  ASSERT_TRUE(core.reset(false, initial));

  std::vector<double> actions(BatchUvmsCore::kActionDim, 0.0);
  actions[BatchUvmsCore::kVehicleActionDim + 1] = 0.38;
  for (std::uint64_t tick = 1; tick <= 200; ++tick) {
    ASSERT_TRUE(core.set_actions(actions, tick));
    core.step(0.005);
  }

  const auto & measured = core.observations();
  EXPECT_GT(measured[BatchUvmsCore::kVehicleStateDim + 1], 0.75);
}

#include "ros2_control_blue_reach_5/arm_integration_policy.hpp"
#include <limits>

TEST(BatchArmIntegration, SharedIntervalPolicyAndRejection)
{
  using ros2_control_blue_reach_5::arm_integration_policy::intervals;
  EXPECT_EQ(intervals(.005, 200.), 1);
  EXPECT_EQ(intervals(.02, 200.), 2);
  EXPECT_EQ(intervals(.1, 200.), 10);
  EXPECT_EQ(intervals(.01, 6000.), 2);
  EXPECT_EQ(intervals(.02, 0.), 2);
  EXPECT_THROW(intervals(0., 200.), std::runtime_error);
  EXPECT_THROW(intervals(5.01, 200.), std::runtime_error);
  EXPECT_THROW(intervals(.01, -1.), std::runtime_error);
  EXPECT_THROW(intervals(.01, std::numeric_limits<double>::infinity()), std::runtime_error);
  EXPECT_THROW(intervals(.01, 1e8), std::runtime_error);
  BatchUvmsCore core;
  core.configure(1);
  const auto before = core.observations();
  EXPECT_THROW(core.step(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
  EXPECT_EQ(core.observations(), before);
  EXPECT_EQ(core.sim_time(), 0.);
}
