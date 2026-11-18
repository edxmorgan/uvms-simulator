#pragma once
#include <array>
#include <cmath>

namespace uvms::arm_identification {
// Completed-step snapshots. Impulse accounts for actuator changes between ROS
// publications.
struct Sample {
  double time = 0, supported = 0;
  std::array<double, 5> position{}, velocity{}, impulse{}, generation{},
      valid{};
  std::array<double, 15> context{0, 0, 9.81, .19, 0, -.12, 3.14159, 0,
                                 0, 0, 0,    0,   0, 0,    0};
  void reset() {
    time = 0;
    supported = 0;
    impulse.fill(0);
    valid.fill(0);
    for (auto &g : generation)
      ++g;
  }
  void advance(double dt, const std::array<double, 5> &q,
               const std::array<double, 5> &v,
               const std::array<double, 5> &torque,
               const std::array<bool, 5> &usable) {
    if (!std::isfinite(dt) || dt <= 0) {
      reset();
      return;
    }
    time += dt;
    for (int j = 0; j < 5; ++j) {
      position[j] = q[j];
      velocity[j] = v[j];
      valid[j] = supported && usable[j] && std::isfinite(q[j]) &&
                 std::isfinite(v[j]) && std::isfinite(torque[j]);
      if (valid[j])
        impulse[j] += dt * torque[j];
      else {
        ++generation[j];
        impulse[j] = 0;
      }
    }
  }
};
} // namespace uvms::arm_identification
