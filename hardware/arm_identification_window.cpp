#include "arm_regressor.h"
#include "ros2_control_blue_reach_5/arm_identifier.hpp"
#include <cmath>
#include <stdexcept>

namespace uvms::arm_identification {
namespace {
void terms(const Sample &s, ArmMatrix &momentum, ArmMatrix &remainder) {
  Eigen::Matrix<double, 4, 48, Eigen::ColMajor> m, b;
  const double *args[] = {s.position.data(), s.velocity.data(),
                          s.context.data(), s.context.data() + 3,
                          s.context.data() + 9};
  double *out[] = {m.data(), b.data()};
  std::vector<double> w(arm_weak_terms_SZ_W);
  std::vector<casadi_int> iw(arm_weak_terms_SZ_IW);
  if (arm_weak_terms(args, out, iw.data(), w.data(), 0))
    throw std::runtime_error("arm regressor failed");
  momentum = m;
  remainder = b;
}
} // namespace
ArmMatrix regressor(const Sample &s, const Eigen::Vector4d &a) {
  Eigen::Matrix<double, 4, 48, Eigen::ColMajor> y;
  const double *args[] = {
      s.position.data(), s.velocity.data(),    a.data(),
      s.context.data(),  s.context.data() + 3, s.context.data() + 9};
  double *out[] = {y.data()};
  std::vector<double> w(arm_regressor_SZ_W);
  std::vector<casadi_int> iw(arm_regressor_SZ_IW);
  if (arm_regressor(args, out, iw.data(), w.data(), 0))
    throw std::runtime_error("arm regressor failed");
  return y;
}
Data stack(const std::vector<Row> &rows) {
  Data d;
  d.rows = rows;
  if (rows.empty())
    return d;
  d.design.resize(rows.size(), rows[0].design.size());
  d.torque.resize(rows.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    d.design.row(i) = rows[i].design;
    d.torque[i] = rows[i].torque;
  }
  return d;
}
Window::Window(double support, double gap, double min_sliding_speed)
    : support_(support), gap_(gap), min_sliding_speed_(min_sliding_speed) {
  if (!std::isfinite(support) || support <= 0 || !std::isfinite(gap) ||
      gap <= 0 || !std::isfinite(min_sliding_speed) || min_sliding_speed < 0)
    throw std::invalid_argument("Invalid arm integration interval");
}
void Window::reset() {
  points_.clear();
  last_row_ = -1;
}
std::vector<Row> Window::observe(const Sample &s) {
  if (!std::isfinite(s.time)) {
    reset();
    return {};
  }
  for (double v : s.context)
    if (!std::isfinite(v)) {
      reset();
      return {};
    }
  for (int j = 0; j < 5; ++j)
    if (!std::isfinite(s.position[j]) || !std::isfinite(s.velocity[j]) ||
        !std::isfinite(s.impulse[j])) {
      reset();
      return {};
    }
  if (!points_.empty()) {
    const auto &p = points_.back().sample;
    if (s.time == p.time)
      return {};
    if (s.time < p.time || s.time - p.time > gap_ || s.context != p.context)
      reset();
  }
  if (!s.supported) {
    reset();
    return {};
  }
  Point p;
  p.sample = s;
  terms(s, p.momentum, p.remainder);
  points_.push_back(std::move(p));
  while (points_.size() > 2 && s.time - points_[1].sample.time >= support_)
    points_.pop_front();
  if (s.time - points_.front().sample.time < support_ ||
      s.time - last_row_ < .099)
    return {};
  last_row_ = s.time;
  const auto &first = points_.front().sample;
  const double duration = s.time - first.time, pi = std::acos(-1.);
  std::vector<Row> rows;
  for (int axis = 0; axis < 5; ++axis) {
    bool valid = true;
    for (const auto &p : points_)
      if (!p.sample.valid[axis] ||
          p.sample.generation[axis] != s.generation[axis] ||
          // sign(v) is not an observed friction force during sticking or
          // unresolved zero-speed chatter. Only use sliding support for arm
          // torque equations; gripper has no Coulomb term.
          (axis < 4 &&
           std::abs(p.sample.velocity[axis]) < min_sliding_speed_)) {
        valid = false;
        break;
      }
    if (!valid)
      continue;
    Row row;
    row.axis = axis;
    row.start = first.time;
    row.end = s.time;
    row.design = Eigen::RowVectorXd::Zero(axis < 4 ? 48 : 3);
    // Hann weak form. The boundary weight is zero; impulse eliminates the need
    // to reconstruct actuator commands at the lower-rate publication times.
    auto integrand = [&](const Point &p) {
      double t = p.sample.time - first.time;
      double w = 1 - std::cos(2 * pi * t / duration),
             dw = 2 * pi / duration * std::sin(2 * pi * t / duration);
      Eigen::RowVectorXd a(row.design.size());
      if (axis < 4)
        a = w * p.remainder.row(axis) - dw * p.momentum.row(axis);
      else
        a << -dw * p.sample.velocity[4], w * p.sample.velocity[4],
            w * p.sample.position[4];
      return std::make_pair(
          a, -dw * (p.sample.impulse[axis] - first.impulse[axis]));
    };
    auto previous = integrand(points_.front());
    for (size_t i = 1; i < points_.size(); ++i) {
      auto current = integrand(points_[i]);
      double h = points_[i].sample.time - points_[i - 1].sample.time;
      row.design += (previous.first + current.first) * (h / (2 * duration));
      row.torque += (previous.second + current.second) * (h / (2 * duration));
      previous = std::move(current);
    }
    if (row.design.allFinite() && std::isfinite(row.torque))
      rows.push_back(std::move(row));
  }
  return rows;
}
} // namespace uvms::arm_identification
