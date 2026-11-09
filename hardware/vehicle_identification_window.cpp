#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include "vehicle_regressor.h"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace uvms::identification {
namespace {
Regressor evaluate(bool bias, const Measurement & m, const Vector6 & delta) {
  casadi_int na, nr, ni, nw;
  if (bias) integral_bias_work(&na, &nr, &ni, &nw);
  else integral_inertia_work(&na, &nr, &ni, &nw);
  std::vector<const double *> args(na, nullptr);
  std::vector<double *> result(nr, nullptr);
  std::vector<casadi_int> iw(ni);
  std::vector<double> work(nw);
  Regressor value;
  if (bias) { args[0] = &m.depth; args[1] = m.attitude.data(); args[2] = m.velocity.data(); }
  else args[0] = delta.data();
  result[0] = value.data();
  int status = bias ? integral_bias(args.data(), result.data(), iw.data(), work.data(), 0)
                    : integral_inertia(args.data(), result.data(), iw.data(), work.data(), 0);
  if (status || !value.allFinite()) throw std::runtime_error("Nonfinite shared regressor");
  return value;
}
}
LiveWindow::LiveWindow(std::size_t horizon, double max_gap)
: horizon_(horizon), max_gap_(max_gap) {
  if (horizon < 5 || !std::isfinite(max_gap) || max_gap <= 0)
    throw std::invalid_argument("Invalid identification window configuration");
}
void LiveWindow::reset() { rows_.clear(); previous_.reset(); older_.reset(); ++epoch_; }
bool LiveWindow::push(const Measurement & m) {
  if (!std::isfinite(m.generation) || !std::isfinite(m.step) || !std::isfinite(m.time) ||
      !std::isfinite(m.period) || !std::isfinite(m.depth) || !m.attitude.allFinite() ||
      !m.velocity.allFinite() || !m.wrench.allFinite() || m.generation < 0 || m.step < 0 ||
      std::floor(m.generation) != m.generation || std::floor(m.step) != m.step ||
      m.generation >= 9007199254740992. || m.step >= 9007199254740992. ||
      m.period <= 0 || m.period > max_gap_) {
    reset(); throw std::invalid_argument("Invalid synchronized measurements");
  }
  if (previous_) {
    const auto & p = *previous_;
    if (m.generation == p.generation && m.step == p.step && m.time == p.time) return false;
    if (m.generation != p.generation || m.step != p.step + 1 ||
        m.time <= p.time || m.time - p.time > max_gap_) reset();
  }
  Regressor bias;
  try { bias = evaluate(true, m, Vector6::Zero()); }
  catch (...) { reset(); throw; }
  bool emitted = false;
  if (previous_ && older_) {
    try {
      double h0=previous_->period, h1=m.period;
      double smooth=(previous_->wrench.array()==m.wrench.array()).all() ? 1. : 0.;
      casadi_int na,nr,ni,nw;
      integral_bias_weights_work(&na,&nr,&ni,&nw);
      std::vector<const double *> args(na,nullptr);
      std::vector<double *> result(nr,nullptr);
      std::vector<casadi_int> iw(ni);
      std::vector<double> work(nw);
      Eigen::Vector3d weights;
      args[0]=&h0;args[1]=&h1;args[2]=&smooth;result[0]=weights.data();
      if(integral_bias_weights(args.data(),result.data(),iw.data(),work.data(),0) ||
         !weights.allFinite()) throw std::runtime_error("Invalid quadrature weights");
      Regressor design = evaluate(false, m, m.velocity - older_->velocity)/(h0+h1)
          + weights[0]*older_bias_ + weights[1]*previous_bias_ + weights[2]*bias;
      Vector6 input=(h0*previous_->wrench+h1*m.wrench)/(h0+h1);
      rows_.push_back({older_->time,m.time,design,input});
      if (rows_.size() > horizon_) rows_.pop_front();
      emitted = true;
    } catch (...) { reset(); throw; }
  }
  older_=previous_; if(previous_) older_bias_=previous_bias_;
  previous_ = m; previous_bias_ = bias;
  return emitted;
}
std::optional<Window> LiveWindow::snapshot() const {
  if (rows_.size() != horizon_) return std::nullopt;
  Window w{epoch_, rows_.front().start, rows_.back().time,
           Eigen::MatrixXd(horizon_ * 6, 27), Eigen::VectorXd(horizon_ * 6), {}};
  for (std::size_t i = 0; i < horizon_; ++i) {
    w.times.push_back(rows_[i].time);
    w.support_starts.push_back(rows_[i].start);
    w.design.middleRows<6>(i * 6) = rows_[i].design;
    w.wrench.segment<6>(i * 6) = rows_[i].wrench;
  }
  return w;
}
Eigen::Matrix<double, 6, 6> inertia_matrix(const Parameters & p) {
  Eigen::Matrix<double, 6, 6> m = Eigen::Matrix<double, 6, 6>::Zero();
  m.diagonal() << p[0], p[1], p[2], p[7], p[8], p[9];
  m(0,4)=p[3]; m(1,3)=p[4]; m(3,1)=p[5]; m(4,0)=p[6];
  return m;
}
}
