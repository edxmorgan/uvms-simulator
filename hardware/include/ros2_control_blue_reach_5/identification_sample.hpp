#pragma once
#include <array>
#include <cmath>

namespace uvms::identification {
// Hardware observation packet. Inputs are body-NED measurements and the modeled
// wrench of the last transmitted PWM, never EKF motion or requested effort.
struct HardwareSample {
  double generation=0, step=0, time=0, period=0, valid=0, depth=0;
  std::array<double,3> attitude{};
  std::array<double,6> velocity{}, wrench{};
  void reset() {
    ++generation;step=0;valid=0;anchored_=false;elapsed_=0;impulse_.fill(0);
  }
  bool advance(double dt,const std::array<double,6> & applied) {
    if(!std::isfinite(dt) || dt<=0 || dt>.5) {reset();return false;}
    for(double x:applied)if(!std::isfinite(x)){reset();return false;}
    clock_+=dt;
    if(anchored_) {
      elapsed_+=dt;
      for(int i=0;i<6;++i)impulse_[i]+=dt*applied[i];
      if(elapsed_>.5){reset();return false;}
    }
    return true;
  }
  bool observe(double z,const std::array<double,3> & angles,const std::array<double,6> & speeds) {
    if(!std::isfinite(z)){reset();return false;}
    for(double x:angles)if(!std::isfinite(x)){reset();return false;}
    for(double x:speeds)if(!std::isfinite(x)){reset();return false;}
    if(anchored_ && elapsed_<=0)return false;
    depth=z;attitude=angles;velocity=speeds;time=clock_;
    // First packet anchors the interval; LiveWindow does not create a row for it.
    period=anchored_?elapsed_:1e-6;
    for(int i=0;i<6;++i)wrench[i]=anchored_?impulse_[i]/elapsed_:0;
    ++step;valid=1;anchored_=true;elapsed_=0;impulse_.fill(0);
    return true;
  }
 private:
  double clock_=0,elapsed_=0;
  bool anchored_=false;
  std::array<double,6> impulse_{};
};
}
