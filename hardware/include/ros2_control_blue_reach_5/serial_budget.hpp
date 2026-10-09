#pragma once

#include <chrono>
#include <algorithm>
#include <stdexcept>

namespace ros2_control_blue_reach_5
{
// 115200 baud, 8N1: 11520 bytes/s in each direction. Five joints:
// requests: 20 * 15 * 7 = 2100 B/s TX; responses: 20 * 15 * 10 = 3000 B/s RX.
// commands: 50 * 5 * 10 = 2500 B/s TX, plus at most 2500 B/s for zero-velocity
// follow-ups. Worst regular TX = 7100 B/s, leaving room for heartbeat/mode traffic.
struct SerialBudget
{
  static constexpr int max_poll_hz = 20;
  static constexpr int max_command_hz = 50;
  static int poll_rate(int requested)
  {
    if (requested <= 0) throw std::invalid_argument("state_update_frequency must be positive");
    return std::min(requested, max_poll_hz);
  }

  using Clock = std::chrono::steady_clock;
  bool command_due(Clock::time_point now)
  {
    if (started_ && now - last_ < std::chrono::milliseconds(20)) return false;
    started_ = true;
    last_ = now;  // No catch-up bursts: the next batch takes the latest commands.
    return true;
  }
  void reset() { started_ = false; }

private:
  bool started_{false};
  Clock::time_point last_{};
};
}
