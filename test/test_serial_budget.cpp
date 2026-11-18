#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/serial_budget.hpp"
#include "ros2_control_blue_reach_5/packet.hpp"
#include "ros2_control_blue_reach_5/cobs.hpp"
#include "ros2_control_blue_reach_5/crc.hpp"
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

TEST(SerialPacket, RejectsTruncatedAndCorruptFrames)
{
  using namespace alpha::driver;
  const Packet packet(PacketId::PacketID_CURRENT, DeviceId::kRotateBase, {1, 2, 3, 4});
  const auto encoded = packet.encode();
  for (std::size_t n = 0; n + 1 < encoded.size(); ++n) {
    std::vector<unsigned char> truncated(encoded.begin(), encoded.begin() + n);
    EXPECT_THROW(Packet::decode(truncated), std::exception);
    truncated.push_back(0);
    EXPECT_THROW(Packet::decode(truncated), std::exception);
  }
  auto missing_delimiter = encoded;
  missing_delimiter.pop_back();
  EXPECT_THROW(Packet::decode(missing_delimiter), std::exception);
  auto corrupt = encoded;
  corrupt[1] ^= 1;
  EXPECT_THROW(Packet::decode(corrupt), std::exception);
  EXPECT_EQ(Packet::decode(encoded).getData(), packet.getData());
  EXPECT_THROW(Packet::decode({1, 1, 0}), std::exception);
  EXPECT_THROW(Packet::decode({255, 1, 0}), std::exception);
}

TEST(SerialPacket, RoundTripsZerosAndLongBlocks)
{
  using namespace alpha::driver;
  for (const auto value : {0, 1, 255}) {
    const Packet packet(PacketId::PacketID_CURRENT, DeviceId::kRotateBase,
                        std::vector<unsigned char>(250, value));
    EXPECT_EQ(Packet::decode(packet.encode()).getData(), packet.getData());
  }
}

TEST(SerialPacket, EnforcesProtocolSizeBeforeLengthCanWrap)
{
  using namespace alpha::driver;
  for (const std::size_t size : {251u, 252u, 256u, 1000u}) {
    EXPECT_THROW((Packet(PacketId::PacketID_CURRENT, DeviceId::kRotateBase,
      std::vector<unsigned char>(size, 1))), std::invalid_argument);
  }
  const Packet maximum(PacketId::PacketID_CURRENT, DeviceId::kRotateBase,
    std::vector<unsigned char>(Packet::kMaxPayloadSize, 1));
  EXPECT_LE(maximum.encode().size(), Packet::kMaxEncodedSize);
  EXPECT_EQ(Packet::decode(maximum.encode()).getData(), maximum.getData());

  // Valid CRC and uint8 length must not bypass the stricter protocol maximum.
  // Zeros keep the frame within the encoded cap, exercising the decoded cap too.
  std::vector<unsigned char> oversized(251, 0);
  oversized.push_back(static_cast<unsigned char>(PacketId::PacketID_CURRENT));
  oversized.push_back(static_cast<unsigned char>(DeviceId::kRotateBase));
  oversized.push_back(255);
  oversized.push_back(calculateReachCrc8(oversized));
  const auto encoded = cobsEncode(oversized);
  ASSERT_LE(encoded.size(), Packet::kMaxEncodedSize);
  EXPECT_THROW(Packet::decode(encoded), std::runtime_error);
  EXPECT_THROW(Packet::decode(std::vector<unsigned char>(Packet::kMaxEncodedSize + 1, 0)),
    std::runtime_error);
}

TEST(SerialPacket, CobsHandlesFullBlocksAndRejectsInteriorDelimiters)
{
  using namespace alpha::driver;
  for (const std::size_t size : {0u, 1u, 253u, 254u, 255u, 508u, 509u}) {
    for (const unsigned char value : {0, 1, 255}) {
      std::vector<unsigned char> data(size, value);
      EXPECT_EQ(cobsDecode(cobsEncode(data)), data);
      data.push_back(0);
      EXPECT_EQ(cobsDecode(cobsEncode(data)), data);
    }
  }
  EXPECT_THROW(cobsDecode({2, 0, 0}), std::runtime_error);
  EXPECT_THROW(cobsDecode({0, 0}), std::runtime_error);
  EXPECT_THROW(cobsDecode({3, 1, 0}), std::runtime_error);
}

TEST(SerialPacket, ReachCrcMatchesIndependentBitwiseCalculation)
{
  using namespace alpha::driver;
  std::vector<unsigned char> data;
  for (unsigned value = 0; value < 256; ++value) {
    data.push_back(static_cast<unsigned char>(value));
    unsigned char crc = 0;
    for (const auto byte : data) {
      crc ^= byte;
      for (int bit = 0; bit < 8; ++bit) {
        // Reflected polynomial corresponding to the declared 0x4D polynomial.
        crc = (crc & 1) ? (crc >> 1) ^ 0xB2 : crc >> 1;
      }
    }
    EXPECT_EQ(calculateReachCrc8(data), static_cast<unsigned char>(crc ^ 0xFF));
  }
}

#include "ros2_control_blue_reach_5/serial_trace.hpp"

TEST(SerialTrace, SeparatesRequestsFromReceptionAndCallbackCompletion)
{
  using namespace alpha::driver;
  SerialTrace trace;
  trace.record(3, PacketId::PacketID_VELOCITY, SerialTrace::Stage::requested, 1000000);
  auto report=trace.report(11000000);
  EXPECT_NE(report.find("device=3/velocity request_written_count=1 age_ms=10"), std::string::npos);
  EXPECT_NE(report.find("decoded_count=0 age_ms=-1"), std::string::npos);
  trace.record(3, PacketId::PacketID_VELOCITY, SerialTrace::Stage::decoded, 2000000);
  report=trace.report(11000000);
  EXPECT_NE(report.find("decoded_count=1 age_ms=9"), std::string::npos);
  EXPECT_NE(report.find("callback_done_count=0 age_ms=-1"), std::string::npos);
  trace.record(3, PacketId::PacketID_VELOCITY, SerialTrace::Stage::dispatched, 3000000);
  EXPECT_NE(trace.report(11000000).find("callback_done_count=1 age_ms=8"), std::string::npos);
}

TEST(SerialTrace, DoesNotLetOneDeviceRefreshAnother)
{
  using namespace alpha::driver;
  SerialTrace trace;
  trace.record(1, PacketId::PacketID_POSITION, SerialTrace::Stage::decoded, 1000000);
  trace.record(5, PacketId::PacketID_POSITION, SerialTrace::Stage::decoded, 490000000);
  auto report=trace.report(501000000);
  EXPECT_NE(report.find("decoded_count=1 age_ms=500"), std::string::npos);
  EXPECT_NE(report.find("decoded_count=1 age_ms=11"), std::string::npos);
  trace.record(256, PacketId::PacketID_POSITION, SerialTrace::Stage::decoded);
}
