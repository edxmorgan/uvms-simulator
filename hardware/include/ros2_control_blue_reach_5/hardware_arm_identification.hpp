#pragma once
#include "ros2_control_blue_reach_5/arm_identification_sample.hpp"
#include <algorithm>
#include <deque>
#include <limits>

namespace uvms::arm_identification {
// Receive-time interpolation, not device-time synchronization. A common watermark
// is bracketed by every channel; torque is integrated through every current sample.
// Called under the hardware's serial-state mutex. No command or model application.
class HardwareMeasurements {
  struct Point { double time, value; };
  using History = std::deque<Point>;
  std::array<std::array<History, 3>, 4> history_;
  double last_ = -1;
  static constexpr double gap_ = .15;
  static double value(const History &h, double t) {
    for (size_t i=1; i<h.size(); ++i)
      if (h[i].time >= t) {
        const auto &a=h[i-1], &b=h[i];
        return a.value+(b.value-a.value)*(t-a.time)/(b.time-a.time);
      }
    return h.back().value;
  }
public:
  Sample sample;
  void reset() {
    for (auto &axis:history_) for(auto &h:axis) h.clear();
    sample.reset(); last_=-1;
  }
  void update(int axis, int field, double time, double x) {
    if (axis<0 || axis>=4 || field<0 || field>=3) return;
    auto &h=history_[axis][field];
    if (!std::isfinite(time) || !std::isfinite(x)) { reset(); return; }
    if (!h.empty() && (time<=h.back().time || time-h.back().time>gap_)) reset();
    h.push_back({time,x});
    if(h.size()>256) { reset(); h.push_back({time,x}); }
  }
  bool snapshot(double now, const std::array<double,4> &lower,
                const std::array<double,4> &upper) {
    double t=now, first=-std::numeric_limits<double>::infinity();
    for(const auto &axis:history_) for(const auto &h:axis) {
      if(h.size()<2) return false;
      if(now<h.back().time || now-h.back().time>gap_) { reset(); return false; }
      t=std::min(t,h.back().time); first=std::max(first,h.front().time);
    }
    if(t<first || t<=last_) return false;
    if(last_>=0 && (last_<first || t-last_>gap_)) { reset(); return false; }
    sample.time=t; sample.supported=1;
    bool constrained=false;
    for(int j=0;j<4;++j) {
      sample.position[j]=value(history_[j][0],t);
      sample.velocity[j]=value(history_[j][1],t);
      constrained |= sample.position[j]<=lower[j]+.01 || sample.position[j]>=upper[j]-.01;
    }
    // A contact on any arm joint changes the coupled dynamics of all four.
    for(int j=0;j<4;++j) {
      sample.valid[j]=!constrained;
      if(constrained) { ++sample.generation[j]; sample.impulse[j]=0; }
      else if(last_>=0) {
        const auto &h=history_[j][2];
        double a=last_, u=value(h,a);
        for(const auto &p:h) if(p.time>a && p.time<t) {
          sample.impulse[j]+=(p.time-a)*(u+p.value)*.5;
          a=p.time; u=p.value;
        }
        sample.impulse[j]+=(t-a)*(u+value(h,t))*.5;
      }
    }
    // Hardware gripper current/force scaling has not been qualified.
    sample.valid[4]=0;
    last_=t;
    for(auto &axis:history_) for(auto &h:axis)
      while(h.size()>2 && h[1].time<=t) h.pop_front();
    return true;
  }
};
}
