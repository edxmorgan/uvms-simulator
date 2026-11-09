#pragma once
#include <cmath>
#include <stdexcept>

#ifdef __CUDACC__
#define UVMS_ARM_HOST_DEVICE __host__ __device__
#else
#define UVMS_ARM_HOST_DEVICE
#endif

namespace ros2_control_blue_reach_5::arm_integration_policy {
// Shared by main plant/EKF scheduling and CPU/CUDA batch arm integration.
// Each plant interval contains 30 RK4 elements. Bound interval duration, not
// elapsed time: alpha*h_element <= 1; the 10 ms cap resolves free-arm dynamics.
// This is an engineering bound, not a guarantee for arbitrary physical models.
UVMS_ARM_HOST_DEVICE inline int count_or_error(double dt, double alpha) {
  if (!std::isfinite(dt) || dt <= 0. || dt > 5. ||
      !std::isfinite(alpha) || alpha < 0.) return -1;
  const double alpha_limit = alpha > 0. ? 30. / alpha : .01;
  const double limit = alpha_limit < .01 ? alpha_limit : .01;
  const double count = std::ceil(dt / limit);
  if (count > 1000.) return -2;
  return static_cast<int>(count);
}
inline int intervals(double dt, double alpha) {
  const int count = count_or_error(dt, alpha);
  if (count == -1)
    throw std::runtime_error("arm integration requires finite 0 < dt <= 5 s and alpha >= 0; reset after longer gaps");
  if (count == -2)
    throw std::runtime_error("arm integration work budget exceeded; reset required");
  return count;
}
}  // namespace ros2_control_blue_reach_5::arm_integration_policy
#undef UVMS_ARM_HOST_DEVICE
