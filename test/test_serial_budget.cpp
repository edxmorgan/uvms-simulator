#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/serial_budget.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
using ros2_control_blue_reach_5::SerialBudget;

TEST(SerialBudget, BudgetUsesActualEncodedPacketSizes)
{
  using namespace alpha::driver;
  const Packet request(PacketId::PacketID_REQUEST_PACKET, DeviceId::kRotateBase, {5});
  const Packet current(PacketId::PacketID_CURRENT, DeviceId::kRotateBase, {0, 0, 0, 0});
  EXPECT_EQ(request.encode().size(), 7u);
  EXPECT_EQ(current.encode().size(), 10u);
}

TEST(SerialBudget, CapsPollingAndRejectsInvalidRates)
{
  EXPECT_EQ(SerialBudget::poll_rate(200), 20);
  EXPECT_EQ(SerialBudget::poll_rate(10), 10);
  EXPECT_THROW(SerialBudget::poll_rate(0), std::invalid_argument);
  EXPECT_THROW(SerialBudget::poll_rate(-1), std::invalid_argument);
  EXPECT_LT(20 * 15 * 7 + 50 * 5 * 10 * 2, 11520 * 0.7);
}

TEST(SerialBudget, ThrottlesWithoutCatchupBursts)
{
  SerialBudget budget;
  const SerialBudget::Clock::time_point start{};
  EXPECT_TRUE(budget.command_due(start));
  EXPECT_FALSE(budget.command_due(start + std::chrono::milliseconds(19)));
  EXPECT_TRUE(budget.command_due(start + std::chrono::milliseconds(20)));
  EXPECT_TRUE(budget.command_due(start + std::chrono::seconds(5)));
  EXPECT_FALSE(budget.command_due(start + std::chrono::seconds(5)));
  budget.reset();
  EXPECT_TRUE(budget.command_due(start + std::chrono::seconds(5)));
}
