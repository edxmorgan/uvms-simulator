#include <gtest/gtest.h>

#include <vector>

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"

using ros2_control_blue_reach_5::BatchUvmsCore;

TEST(BatchArmActuation, SubStaticThresholdCommandMovesReleasedJoint)
{
  BatchUvmsCore core;
  core.configure(1);
  std::vector<float> initial(BatchUvmsCore::kObservationDim, 0.0F);
  initial[BatchUvmsCore::kVehicleStateDim + 0] = 3.1F;
  initial[BatchUvmsCore::kVehicleStateDim + 1] = 0.7F;
  initial[BatchUvmsCore::kVehicleStateDim + 2] = 0.4F;
  initial[BatchUvmsCore::kVehicleStateDim + 3] = 2.1F;
  initial[BatchUvmsCore::kVehicleStateDim + 4] = 0.006F;
  ASSERT_TRUE(core.reset(false, initial));

  std::vector<float> actions(BatchUvmsCore::kActionDim, 0.0F);
  actions[BatchUvmsCore::kVehicleActionDim + 1] = 0.38F;
  for (std::uint64_t tick = 1; tick <= 200; ++tick) {
    ASSERT_TRUE(core.set_actions(actions, tick));
    core.step(0.005);
  }

  const auto & measured = core.observations();
  EXPECT_GT(measured[BatchUvmsCore::kVehicleStateDim + 1], 0.75F);
}
