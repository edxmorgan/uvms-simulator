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
LiveWindow::LiveWindow(std::size_t horizon, double max_gap, double integration_seconds)
: horizon_(horizon), max_gap_(max_gap), integration_seconds_(integration_seconds) {
  if (horizon < 5 || !std::isfinite(max_gap) || max_gap <= 0 ||
      !std::isfinite(integration_seconds) || integration_seconds < 0)
    throw std::invalid_argument("Invalid identification window configuration");
}
void LiveWindow::reset() { rows_.clear(); observations_.clear(); previous_.reset(); older_.reset(); next_row_time_=0.; ++epoch_; }
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
  if(integration_seconds_>0) {
    observations_.push_back({m,bias});
    while(observations_.size()>4 && m.time-observations_[1].measurement.time>=integration_seconds_)
      observations_.pop_front();
    previous_=m;
    if(observations_.size()<4 || m.time-observations_.front().measurement.time<integration_seconds_)return false;
    // Quantize a 10 Hz row schedule to the nearest observation. A strict
    // elapsed >= 0.1 check would discard normal 10 Hz samples under clock jitter.
    if(rows_.empty())next_row_time_=m.time;
    if(m.time+.5*std::min(m.period,.1)+1e-9<next_row_time_)return false;
    do {next_row_time_+=.1;} while(next_row_time_<=m.time);
    const double start=observations_.front().measurement.time, duration=m.time-start;
    Regressor design=Regressor::Zero();Vector6 input=Vector6::Zero();
    double normalization=0.;
    // Weak momentum balance, integrated by parts. Apply the same quadrature
    // to measured velocity and accumulated actuator impulse. Averaging force
    // independently inside each sensor interval loses its correlation with
    // the Hann weight and biases fits when the input changes rapidly.
    const std::size_t count=observations_.size();
    std::vector<double> quadrature(count,0.);
    const std::size_t intervals=count-1;
    const std::size_t paired=intervals-(intervals%2?3:0);
    for(std::size_t i=0;i<paired;i+=2) {
      const double h0=observations_[i+1].measurement.time-observations_[i].measurement.time;
      const double h1=observations_[i+2].measurement.time-observations_[i+1].measurement.time;
      const double h=h0+h1;
      quadrature[i]+=h*(2*h0-h1)/(6*h0);
      quadrature[i+1]+=h*h*h/(6*h0*h1);
      quadrature[i+2]+=h*(2*h1-h0)/(6*h1);
    }
    if(intervals%2) {
      // Integrate the final three intervals with a cubic rule, including
      // nonuniform timestamps; no unmatched low-order endpoint interval.
      Eigen::Matrix4d powers;Eigen::Vector4d moments;
      const double origin=observations_[paired].measurement.time;
      const double h=m.time-origin;
      for(int k=0;k<4;++k) {
        moments[k]=h/(k+1);
        for(int j=0;j<4;++j)
          powers(k,j)=std::pow((observations_[paired+j].measurement.time-origin)/h,k);
      }
      const Eigen::Vector4d weights=powers.fullPivLu().solve(moments);
      for(int j=0;j<4;++j)quadrature[paired+j]+=weights[j];
    }
    Vector6 impulse=Vector6::Zero();
    const Vector6 initial_velocity=observations_.front().measurement.velocity;
    const double pi=std::acos(-1.);
    for(std::size_t i=0;i<count;++i) {
      const auto & observation=observations_[i];
      if(i)impulse+=(observation.measurement.time-observations_[i-1].measurement.time)*observation.measurement.wrench;
      const double phase=2*pi*(observation.measurement.time-start)/duration;
      const double weight=.5*(1.-std::cos(phase));
      const double derivative=pi/duration*std::sin(phase);
      design+=quadrature[i]*(weight*observation.bias-derivative*
        evaluate(false,m,observation.measurement.velocity-initial_velocity));
      input-=quadrature[i]*derivative*impulse;
      normalization+=quadrature[i]*weight;
    }
    if(!std::isfinite(normalization) || normalization<=0 || !design.allFinite() || !input.allFinite()) {
      reset();throw std::runtime_error("Invalid weighted momentum quadrature");
    }
    rows_.push_back({start,m.time,design/normalization,input/normalization});
    if(rows_.size()>horizon_)rows_.pop_front();
    return true;
  }
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
           Eigen::MatrixXd(horizon_ * 6, 27), Eigen::VectorXd(horizon_ * 6), {}, {}};
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
