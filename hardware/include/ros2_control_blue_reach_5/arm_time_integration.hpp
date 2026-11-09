#pragma once
#include "ros2_control_blue_reach_5/arm_integration_policy.hpp"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ros2_control_blue_reach_5::arm_time_integration {
using arm_integration_policy::intervals;
inline void require_finite(const casadi::DM &a) {
  for (double v:a.nonzeros())
    if (!std::isfinite(v)) throw std::runtime_error("nonfinite arm integration result");
}
inline std::vector<casadi::DM> plant(const casadi::Function &flow,
                                    std::vector<casadi::DM> args) {
  const double dt=double(args.at(2));
  const int count=intervals(dt,double(args.at(8)));
  args[2]=dt/count;
  std::vector<casadi::DM> result;
  for(int k=0;k<count;++k) {
    result=flow(args); require_finite(result.at(0)); args[0]=result[0];
  }
  // Preserve the existing end-of-completed-step joint-limit projection.
  return result;
}
inline std::vector<casadi::DM> posterior(const casadi::Function &predict,
                                       const casadi::Function &correct,
                                       std::vector<casadi::DM> args) {
  const double dt=double(args.at(4));
  const int count=intervals(dt,double(args.at(9)));
  const double h=dt/count;
  args[4]=h;
  args[6]=args[6]/count; // caller supplies Q_d for the full completed interval
  for(int k=1;k<count;++k) {
    auto step=predict(std::vector<casadi::DM>{args[0],args[2],args[3],h,args[8],args[9]});
    require_finite(step.at(0)); require_finite(step.at(1));
    args[0]=step[0];
    args[1]=casadi::DM::mtimes(std::vector<casadi::DM>{step[1],args[1],step[1].T()})+args[6];
    args[1]=(args[1]+args[1].T())*.5;
    require_finite(args[1]);
  }
  // Exactly one observation update. No interpolation or synthetic measurements.
  return correct(args);
}
}
