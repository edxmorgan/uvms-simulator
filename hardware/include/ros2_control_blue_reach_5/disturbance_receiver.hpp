#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_blue_reach_5/msg/disturbance_command.hpp"
#include "ros2_control_blue_reach_5/msg/disturbance_state.hpp"

namespace ros2_control_blue_reach_5 {
// Shared receiver contract for simulated hardware only. No estimator inputs.
class DisturbanceLease {
public:
  using Command = msg::DisturbanceCommand;
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  explicit DisturbanceLease(const std::string &prefix) : frame_(prefix + "base_link") {
    for (size_t i=0; i<4; ++i) names_[i] = prefix + "_axis_" + std::string(1, "edcb"[i]);
  }
  static std::array<double, 6> wrench(const Command &m) {
    return {m.body_wrench.force.x, m.body_wrench.force.y, m.body_wrench.force.z,
            m.body_wrench.torque.x, m.body_wrench.torque.y, m.body_wrench.torque.z};
  }
  bool active(Time now) const { return !owner_.empty() && now-received_ < std::chrono::seconds(1); }
  bool receive(const Command &m, Time now) {
    auto reject = [&](const char *why) { rejection_ = why; return false; };
    if (m.sequence==0 || m.session_id.empty() || m.session_id.size()>128 || m.header.frame_id!=frame_ || m.joint_names!=names_)
      return reject("Invalid session, body frame or joint names");
    bool zero=true;
    for(double v:wrench(m)) { if(!std::isfinite(v)) return reject("Nonfinite wrench"); zero &= v==0.; }
    for(double v:m.joint_torques) { if(!std::isfinite(v)) return reject("Nonfinite torque"); zero &= v==0.; }
    if (m.acquire && (!zero || m.release)) return reject("Acquisition must be zero and not release");
    if (closed_.count(m.session_id)) {
      if(m.release) { rejection_.clear(); return true; } // idempotent release
      return reject("Closed session");
    }
    if (!owner_.empty() && !active(now)) { closed_.insert(owner_); owner_.clear(); }
    if (closed_.count(m.session_id)) return reject("Expired session");
    if (!owner_.empty() && owner_!=m.session_id) return reject("Injection owned by another session");
    if (owner_.empty()) {
      if (!m.acquire) return reject("Acquire before sending loads");
      owner_=m.session_id; sequence_=0;
    }
    if (m.sequence<=sequence_) return reject("Non-increasing sequence");
    sequence_=m.sequence; received_=now; command_=m;
    if(m.release) { closed_.insert(owner_); owner_.clear(); }
    rejection_.clear(); return true;
  }
  Command sample(Time now) const {
    if(active(now)) return command_;
    Command zero; zero.header.frame_id=frame_; zero.joint_names=names_; return zero;
  }
  void clear() { if(!owner_.empty()) closed_.insert(owner_); owner_.clear(); command_=Command{}; }
  std::string owner(Time now) const { return active(now)?owner_:""; }
  std::string rejection() const { return rejection_; }
  const std::array<std::string,4> &names() const { return names_; }
private:
  std::string frame_,owner_,rejection_;
  std::unordered_set<std::string> closed_;
  std::array<std::string,4> names_;
  uint64_t sequence_=0;
  Time received_{};
  Command command_;
};

class DisturbanceReceiver {
public:
  using Command = msg::DisturbanceCommand;
  DisturbanceReceiver(rclcpp::Node::SharedPtr node, const std::string &prefix,
                      const std::string &component) : node_(node), lease_(prefix) {
    state_.component=component; state_.header.frame_id=prefix+"base_link";
    state_.joint_names=lease_.names();
    const auto base="/"+prefix+"/disturbance/";
    sub_=node_->create_subscription<Command>(base+"command",rclcpp::QoS(1).reliable(),
      [this](Command::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_); lease_.receive(*m,DisturbanceLease::Clock::now());
      });
    pub_=node_->create_publisher<msg::DisturbanceState>(base+component+"/state",rclcpp::QoS(1).reliable());
    timer_=node_->create_wall_timer(std::chrono::milliseconds(50),[this] {
      msg::DisturbanceState out;
      { std::lock_guard<std::mutex> lock(mutex_); out=state_;
        const auto now=DisturbanceLease::Clock::now();
        out.lease_owner=lease_.owner(now); out.lease_active=lease_.active(now);
        out.last_rejection=lease_.rejection(); }
      out.header.stamp=node_->now(); pub_->publish(out);
    });
  }
  Command sample() { std::lock_guard<std::mutex> lock(mutex_); return lease_.sample(DisturbanceLease::Clock::now()); }
  void paused(bool paused) { std::lock_guard<std::mutex> lock(mutex_); state_.paused=paused; }
  void completed(const Command &m,double dt, const std::array<double,6> &contact = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.dynamics_time+=dt; state_.step_duration=dt; ++state_.step;
    state_.applied_session=m.session_id; state_.applied_sequence=m.sequence;
    if(state_.component=="vehicle") {
      state_.applied_body_wrench=m.body_wrench;
      state_.contact_body_wrench.force.x=contact[0]; state_.contact_body_wrench.force.y=contact[1];
      state_.contact_body_wrench.force.z=contact[2]; state_.contact_body_wrench.torque.x=contact[3];
      state_.contact_body_wrench.torque.y=contact[4]; state_.contact_body_wrench.torque.z=contact[5];
    }
    else state_.applied_joint_torques=m.joint_torques;
  }
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_); lease_.clear();
    ++state_.generation; state_.step=0; state_.dynamics_time=0.; state_.step_duration=0.;
    state_.applied_session.clear(); state_.applied_sequence=0;
    state_.contact_body_wrench=geometry_msgs::msg::Wrench{};
    state_.applied_body_wrench=geometry_msgs::msg::Wrench{}; state_.applied_joint_torques.fill(0.);
  }
private:
  rclcpp::Node::SharedPtr node_;
  DisturbanceLease lease_;
  std::mutex mutex_;
  msg::DisturbanceState state_;
  rclcpp::Subscription<Command>::SharedPtr sub_;
  rclcpp::Publisher<msg::DisturbanceState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
