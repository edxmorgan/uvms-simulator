#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include "ros2_control_blue_reach_5/packet_id.hpp"

namespace alpha::driver
{
// Receive and transmit threads update independent atomic observations. Reports
// are approximate concurrent snapshots, not request/response sequence matching.
class SerialTrace
{
public:
  enum class Stage { requested, decoded, dispatched };
  static std::int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void record(unsigned device, PacketId packet, Stage stage, std::int64_t now = now_ns())
  {
    const int field = field_index(packet);
    if (device >= channels_.size() || field < 0) return;
    auto &sample = channels_[device][field][static_cast<unsigned>(stage)];
    sample.time.store(now, std::memory_order_relaxed);
    sample.count.fetch_add(1, std::memory_order_relaxed);
  }
  std::string report(std::int64_t now = now_ns()) const
  {
    std::ostringstream out;
    const char *fields[] = {"position", "velocity", "current"};
    const char *stages[] = {"request_written", "decoded", "callback_done"};
    for (unsigned device=0; device<channels_.size(); ++device) {
      for (unsigned field=0; field<3; ++field) {
        const auto &channel=channels_[device][field];
        if (!channel[0].count.load() && !channel[1].count.load()) continue;
        out << " device=" << device << '/' << fields[field];
        for (unsigned stage=0; stage<3; ++stage) {
          const auto count=channel[stage].count.load(std::memory_order_relaxed);
          const auto stamp=channel[stage].time.load(std::memory_order_relaxed);
          out << ' ' << stages[stage] << "_count=" << count
              << " age_ms=" << (count ? (now-stamp)/1e6 : -1.);
        }
      }
    }
    return out.str();
  }
private:
  static int field_index(PacketId packet)
  {
    if (packet==PacketId::PacketID_POSITION) return 0;
    if (packet==PacketId::PacketID_VELOCITY) return 1;
    if (packet==PacketId::PacketID_CURRENT) return 2;
    return -1;
  }
  struct Sample { std::atomic<std::uint64_t> count{0}; std::atomic<std::int64_t> time{0}; };
  std::array<std::array<std::array<Sample,3>,3>,256> channels_{};
};
}
