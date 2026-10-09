#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/command_safety.hpp"
using namespace ros2_control_blue_reach_5;

TEST(CommandSafety, UnrelatedSwitchDoesNotActivateArm)
{
  const std::vector<std::string> names{"arm_e", "arm_d"}, off{"", ""};
  EXPECT_EQ(plan_command_switch(names, off, {"vehicle/force.x"}, {}), off);
  EXPECT_EQ(plan_command_switch(names, off, {"arm_e/effort"}, {}),
    (std::vector<std::string>{"effort", ""}));
  EXPECT_EQ(off[0], "");  // prepare must not mutate active state
}

TEST(CommandSafety, RequiresStopBeforeReclaimAndRejectsUnsupportedModes)
{
  const std::vector<std::string> names{"arm_e"}, on{"effort"};
  EXPECT_THROW(plan_command_switch(names, on, {"arm_e/current"}, {}), std::invalid_argument);
  EXPECT_EQ(plan_command_switch(names, on, {"arm_e/current"}, {"arm_e/effort"}),
    (std::vector<std::string>{"current"}));
  EXPECT_EQ(plan_command_switch(names, on, {}, {"arm_e/effort"}),
    (std::vector<std::string>{""}));
  EXPECT_EQ(plan_command_switch(names, on, {"arm_e/effort"}, {"arm_e/effort"}), on);
  EXPECT_THROW(plan_command_switch(names, {""}, {"arm_e/position"}, {}), std::invalid_argument);
  EXPECT_THROW(plan_command_switch(names, {""}, {"arm_e/velocity"}, {}), std::invalid_argument);
  EXPECT_THROW(plan_command_switch(names, {""}, {"arm_e/acceleration"}, {}), std::invalid_argument);
  EXPECT_THROW(plan_command_switch(names, {""}, {"arm_e/effort", "arm_e/effort"}, {}), std::invalid_argument);
  EXPECT_THROW(plan_command_switch(names, on, {}, {"arm_e/current"}), std::invalid_argument);
  EXPECT_THROW(plan_command_switch(names, {""}, {"arm_e/computed_effort"}, {}), std::invalid_argument);
}

TEST(CommandSafety, RequiresFreshPacketsForEveryField)
{
  JointFeedbackClock feedback;
  const JointFeedbackClock::Clock::time_point t{};
  EXPECT_FALSE(feedback.fresh(t));
  feedback.update(0, t);
  feedback.update(1, t);
  EXPECT_FALSE(feedback.fresh(t));
  feedback.update(2, t);
  EXPECT_TRUE(feedback.fresh(t + std::chrono::milliseconds(500)));
  EXPECT_FALSE(feedback.fresh(t + std::chrono::milliseconds(501)));
  feedback.update(2, t + std::chrono::seconds(1));
  EXPECT_FALSE(feedback.fresh(t + std::chrono::seconds(1))); // current alone is insufficient
  feedback.update(0, t + std::chrono::seconds(1));
  feedback.update(1, t + std::chrono::seconds(1));
  EXPECT_TRUE(feedback.fresh(t + std::chrono::seconds(1)));
  feedback.seen[0] = false;
  EXPECT_FALSE(feedback.fresh(t + std::chrono::seconds(1)));
}
