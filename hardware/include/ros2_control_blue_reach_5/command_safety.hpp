#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace ros2_control_blue_reach_5
{
// Validate without side effects. Native position/velocity need a separately
// validated firmware-limit setup; do not silently reinterpret them as torque.
inline std::vector<std::string> plan_command_switch(
  const std::vector<std::string> &names, const std::vector<std::string> &active,
  const std::vector<std::string> &start, const std::vector<std::string> &stop)
{
  auto result = active;
  const auto apply = [&](const std::string &key, bool starting) {
    const auto slash = key.rfind('/');
    if (slash == std::string::npos) return;
    const auto joint = std::find(names.begin(), names.end(), key.substr(0, slash));
    if (joint == names.end()) return;  // another hardware component
    const auto i = std::distance(names.begin(), joint);
    const auto mode = key.substr(slash + 1);
    if (mode != "effort" && mode != "current")
      throw std::invalid_argument("Unsupported Reach command interface: " + key);
    if (starting) {
      if (!result[i].empty()) throw std::invalid_argument("Conflicting Reach command claim: " + key);
      result[i] = mode;
    } else {
      if (result[i] != mode) throw std::invalid_argument("Stopping unclaimed Reach interface: " + key);
      result[i].clear();
    }
  };
  for (const auto &key : stop) apply(key, false);
  for (const auto &key : start) apply(key, true);
  return result;
}

struct JointFeedbackClock
{
  using Clock = std::chrono::steady_clock;
  std::array<Clock::time_point, 3> received{};
  std::array<bool, 3> seen{false, false, false};
  void update(std::size_t field, Clock::time_point now) { received[field] = now; seen[field] = true; }
  bool fresh(Clock::time_point now) const
  {
    for (std::size_t i = 0; i < seen.size(); ++i)
      if (!seen[i] || now - received[i] > std::chrono::milliseconds(500)) return false;
    return true;
  }
};
}
