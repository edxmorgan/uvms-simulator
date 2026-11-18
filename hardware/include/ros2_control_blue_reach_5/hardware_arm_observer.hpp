#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include "ros2_control_blue_reach_5/arm_time_integration.hpp"
#include "ros2_control_blue_reach_5/command_safety.hpp"
#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"

namespace ros2_control_blue_reach_5 {
// Shadow-mode adapter for the SAME generated augmented arm EKF as simulation.
// Receive timestamps are not device timestamps: bound skew and never assimilate
// the same serial sample twice. No command interface or compensation is provided.
struct HardwareArmObserver {
  using Clock = JointFeedbackClock::Clock;
  casadi::Function update, predict;
  casadi::DM x, P, Q, R, parameters;
  std::array<double, 4> torque{}, variance{}, at_limit{}, locked{}, admitted{};
  std::array<double, 81> model{};
  std::array<double, 16> covariance{}; // row-major disturbance covariance, arm edcb
  std::array<std::array<Clock::time_point, 3>, 4> consumed{};
  Clock::time_point last{};
  bool seeded{false};
  double valid{0}, step{0}, generation{0}, sample_step{0};

  void refresh_disturbance_covariance() {
    for(int i=0;i<4;++i) {
      variance[i]=double(P(8+i,8+i));
      for(int j=0;j<4;++j) covariance[4*i+j]=double(P(8+i,8+j));
    }
  }

  void reset() {
    x = casadi::DM::zeros(12,1); P = casadi::DM::eye(12)*0.001;
    Q = casadi::DM::zeros(12,12); R = casadi::DM::zeros(8,8);
    for (int i=0;i<4;++i) {
      P(8+i,8+i)=1.; Q(i,i)=1e-8; Q(4+i,4+i)=1e-6;
      Q(8+i,8+i)=0.001; R(i,i)=1e-6; R(4+i,4+i)=1e-5;
    }
    parameters = casadi::DM::vertcat(pack_manipulator_dynamics(default_manipulator_dynamics()));
    for (int i=0;i<81;++i) model[i]=double(parameters(i));
    torque.fill(0); refresh_disturbance_covariance(); at_limit.fill(0); admitted.fill(0);
    seeded=false; valid=0; step=sample_step=0; ++generation; consumed={};
  }

  // Returns true only for a new posterior. Endpoint-average measured torque is
  // an approximation to the interval input; hardware calibration remains required.
  casadi::DM previous_input = casadi::DM::zeros(4,1);
  bool observe(const std::array<JointFeedbackClock,4> &clocks,
               const casadi::DM &y, const casadi::DM &input,
               const casadi::DM &lower, const casadi::DM &upper,
               Clock::time_point now) {
    auto oldest=now, newest=Clock::time_point::min();
    bool all_new=true;
    for (int i=0;i<4;++i) for(int j=0;j<3;++j) {
      const auto t=clocks[i].received[j];
      if (!clocks[i].seen[j] || t>now || now-t>std::chrono::milliseconds(150)) {
        valid=0; seeded=false; return false;
      }
      oldest=std::min(oldest,t); newest=std::max(newest,t);
      all_new &= t>consumed[i][j];
    }
    if (newest-oldest>std::chrono::milliseconds(50)) {
      valid=0; seeded=false; return false;
    }
    if (!all_new) return false;
    for (auto v:y.nonzeros()) if(!std::isfinite(v)) { valid=0; seeded=false; return false; }
    for (auto v:input.nonzeros()) if(!std::isfinite(v)) { valid=0; seeded=false; return false; }
    bool constrained=false;
    for(int i=0;i<4;++i) {
      // A guard band around Xacro limits is NOT a new joint limit. Do not use
      // simulated clamp/contact truth on hardware or infer external torque here.
      at_limit[i] = double(y(i))<=double(lower(i))+0.01 || double(y(i))>=double(upper(i))-0.01;
      constrained |= at_limit[i]!=0;
    }
    const double dt=std::chrono::duration<double>(newest-last).count();
    for(int i=0;i<4;++i) consumed[i]=clocks[i].received;
    last=newest; ++sample_step;
    if (!seeded || dt<=0 || dt>0.1) {
      for(int i=0;i<8;++i) x(i)=y(i);
      P=casadi::DM::eye(12)*0.001;
      for(int i=0;i<4;++i) { x(8+i)=0.; P(8+i,8+i)=1.; torque[i]=0.; variance[i]=1.; }
      refresh_disturbance_covariance();
      previous_input=input; seeded=true; valid=0; return false;
    }
    if (constrained) {
      // Unknown mechanical reaction: no invented measurement information.
      // Re-seed after release; keep the displayed uncertainty honest meanwhile.
      P += Q*dt;
      refresh_disturbance_covariance();
      valid=0; seeded=false; previous_input=input; return false;
    }
    try {
      auto out=arm_time_integration::posterior(predict,update,std::vector<casadi::DM>{x,P,(previous_input+input)*0.5,parameters,
        dt,y,Q*dt,R,casadi::DM::zeros(4,1),200.,lower,upper,casadi::DM::zeros(4,1)});
      for (const auto &a:out) for(auto v:a.nonzeros())
        if(!std::isfinite(v)) throw std::runtime_error("nonfinite hardware arm posterior");
      for(int i=0;i<12;++i) if(double(out.at(1)(i,i))<0)
        throw std::runtime_error("negative hardware arm covariance");
      x=out.at(0); P=out.at(1);
      for(int i=0;i<4;++i) torque[i]=double(x(8+i));
      refresh_disturbance_covariance();
      previous_input=input; valid=1; ++step;
      // admitted stays zero: physical torque/contact calibration not validated.
      return true;
    } catch (const std::exception &) {
      valid=0; seeded=false; return false;
    }
  }
};

// Latest-snapshot mailbox for the shadow observer. No actuator or controller
// state is owned here. Numerical work never holds the mailbox mutex; read()
// only tries the lock and keeps its last completed, age-checked diagnostics.
class HardwareArmObserverWorker {
 public:
  using Clock = HardwareArmObserver::Clock;
  struct Sample {
    std::array<JointFeedbackClock,4> clocks;
    std::array<double,8> measurement{};
    std::array<double,4> input{}, lower{}, upper{};
  };
  struct Result {
    std::array<double,4> torque, variance, at_limit, locked, admitted;
    std::array<double,16> covariance;
    double valid, step, sample_step;
    Clock::time_point oldest;
    uint64_t fault_epoch;
    void apply(HardwareArmObserver &out, Clock::time_point now) const {
      out.torque=torque; out.variance=variance; out.at_limit=at_limit;
      out.locked=locked; out.admitted=admitted; out.covariance=covariance;
      out.step=step; out.sample_step=sample_step;
      out.valid=(now>=oldest && now-oldest<=std::chrono::milliseconds(150)) ? valid : 0.;
    }
  };
  explicit HardwareArmObserverWorker(const HardwareArmObserver &observer)
    : observer_(observer), thread_([this]{run();}) {}
  ~HardwareArmObserverWorker() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
    wake_.notify_one();
    thread_.join();
  }
  void exchange(const Sample &sample, HardwareArmObserver &output, Clock::time_point now) {
    bool invalid=false;
    auto oldest=now, newest=Clock::time_point::min();
    for(const auto &clock:sample.clocks) for(int j=0;j<3;++j) {
      oldest=std::min(oldest,clock.received[j]); newest=std::max(newest,clock.received[j]);
      if(!clock.seen[j] || clock.received[j]>now || now-clock.received[j]>std::chrono::milliseconds(150))
        invalid=true;
    }
    if(newest-oldest>std::chrono::milliseconds(50)) invalid=true;
    for(double v:sample.measurement) if(!std::isfinite(v)) invalid=true;
    for(double v:sample.input) if(!std::isfinite(v)) invalid=true;
    // Faults cannot be coalesced away with pending samples. This epoch also
    // rejects a valid result that was already being computed before the fault.
    if(invalid) fault_epoch_.fetch_add(1);
    const auto epoch=fault_epoch_.load();
    {
      std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
      if(lock.owns_lock()) {
        pending_=Pending{sample,epoch};
        if(result_) {
          if(result_->fault_epoch==epoch) {
            result_->apply(output,now); result_oldest_=result_->oldest;
          }
          result_.reset();
        }
      }
    }
    wake_.notify_one();
    // Never retain validity through an outage, even if the worker is busy.
    if(invalid || now<result_oldest_ || now-result_oldest_>std::chrono::milliseconds(150)) output.valid=0.;
  }
 private:
  struct Pending { Sample sample; uint64_t fault_epoch; };
  void run() {
    uint64_t processed_epoch=0;
    while(true) {
      Pending pending;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this]{return stopping_ || pending_.has_value();});
        if(stopping_) return;
        pending=*pending_; pending_.reset();
      }
      const auto &sample=pending.sample;
      if(pending.fault_epoch!=processed_epoch) {
        // Match observe()'s invalid-feedback transition without resetting the
        // session counters or treating a duplicate packet as a new sample.
        observer_.valid=0.; observer_.seeded=false;
        processed_epoch=pending.fault_epoch;
      }
      auto dm=[](const auto &values) {return casadi::DM(std::vector<double>(values.begin(),values.end()));};
      observer_.observe(sample.clocks,dm(sample.measurement),dm(sample.input),
                        dm(sample.lower),dm(sample.upper),Clock::now());
      auto oldest=Clock::now();
      for(const auto &clock:sample.clocks) for(const auto &stamp:clock.received)
        oldest=std::min(oldest,stamp);
      Result result{observer_.torque,observer_.variance,observer_.at_limit,observer_.locked,
                    observer_.admitted,observer_.covariance,observer_.valid,observer_.step,
                    observer_.sample_step,oldest,pending.fault_epoch};
      { std::lock_guard<std::mutex> lock(mutex_); result_=result; }
    }
  }
  HardwareArmObserver observer_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<Pending> pending_;
  std::optional<Result> result_;
  Clock::time_point result_oldest_{};
  bool stopping_{false};
  std::atomic<uint64_t> fault_epoch_{0};
  std::thread thread_;
};

}
