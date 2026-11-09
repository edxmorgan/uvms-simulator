#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uvms::identification {
namespace {
Vector6 residual_rms(const Eigen::MatrixXd & a, const Eigen::VectorXd & b,
                     const Parameters & p) {
  const Eigen::VectorXd residual=a*p-b;
  Vector6 sum=Vector6::Zero();
  for(Eigen::Index i=0;i<residual.size();++i)sum[i%6]+=residual[i]*residual[i];
  return (sum/double(residual.size()/6)).cwiseSqrt();
}
}
RetentionConfig::RetentionConfig() {
  parameter_scales << 50,50,100,5,2,2,5,2,2,5,100,100,1,1,5,
                      100,50,100,5,5,5,10,10,100,1,1,5;
  residual_scales << .1,.1,.1,.01,.01,.01;
}
ModelRetention::ModelRetention(RetentionConfig config, bool fixed_weight)
: config_(std::move(config)), fixed_weight_(fixed_weight) {
  const auto & c=config_;
  if(!c.parameter_scales.allFinite() || c.parameter_scales.minCoeff()<=0 ||
     !c.residual_scales.allFinite() || c.residual_scales.minCoeff()<=0 ||
     !std::isfinite(c.min_information) || c.min_information<=0 ||
     !std::isfinite(c.validation_seconds) || c.validation_seconds<=0 ||
     !std::isfinite(c.validation_gap) || c.validation_gap<0 ||
     !std::isfinite(c.max_validation_error) || c.max_validation_error<=0 ||
     !std::isfinite(c.change_error) || c.change_error<=c.max_validation_error ||
     !std::isfinite(c.improvement_fraction) || c.improvement_fraction<=0 || c.improvement_fraction>=1 ||
     !std::isfinite(c.information_gain) || c.information_gain<=1 || c.change_blocks<1)
    throw std::invalid_argument("Invalid retention scaling/thresholds");
}
Information ModelRetention::information(const Eigen::MatrixXd & a) const {
  if(a.cols()!=27 || a.rows()<30 || a.rows()%6 || !a.allFinite())
    throw std::invalid_argument("Invalid retention design");
  Information out;
  // Only remove the known W+B null direction, and only when the actual
  // regressor confirms it. Never drop weak inertia/drag directions by SVD.
  out.net_weight_only=!fixed_weight_ &&
    (a.col(10)+a.col(11)).norm()<=1e-10*std::max(1.,a.col(10).norm()+a.col(11).norm());
  out.dimensions=(fixed_weight_ || out.net_weight_only)?24:25;
  Eigen::MatrixXd basis=Eigen::MatrixXd::Zero(27,out.dimensions);
  for(int i=0,j=0;i<27;++i) {
    if(i==5 || i==6 || (i==10 && (fixed_weight_ || out.net_weight_only)))continue;
    basis(i,j)=config_.parameter_scales[i];
    if(i==3)basis(6,j)=basis(i,j);
    if(i==4)basis(5,j)=basis(i,j);
    if(i==11 && out.net_weight_only) {
      basis(10,j)=config_.parameter_scales[10]*.5;
      basis(11,j)=-config_.parameter_scales[10]*.5;
    }
    ++j;
  }
  Eigen::MatrixXd normalized=a*basis;
  for(Eigen::Index i=0;i<a.rows();++i)normalized.row(i)/=config_.residual_scales[i%6];
  // Average information per six-axis sample, so longer/repeated windows do
  // not inflate the score. Correlations are NOT assumed to imply confidence.
  normalized/=std::sqrt(double(a.rows()/6));
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized);
  const auto singular=svd.singularValues();
  out.minimum=singular.tail(1)[0]*singular.tail(1)[0];
  const double maximum=singular[0]*singular[0];
  out.condition=out.minimum>0?maximum/out.minimum:std::numeric_limits<double>::infinity();
  // Excitation is an absolute, engineering-scaled sensitivity requirement.
  // A strong restoring-force column must not raise the requirement for drag.
  // Separately reject directions below floating-point numerical resolution.
  const double numerical_tolerance=std::numeric_limits<double>::epsilon()*
    std::max(normalized.rows(),normalized.cols())*singular[0];
  const double cutoff=std::max(config_.min_information,numerical_tolerance*numerical_tolerance);
  out.rank=(singular.array().square()>=cutoff).count();
  out.sufficient=out.rank==out.dimensions;
  return out;
}
double ModelRetention::error(const Window & w,const Parameters & p) const {
  // Require every axis to pass; strong force channels cannot hide bad torques.
  return residual_rms(w.design,w.wrench,p).cwiseQuotient(config_.residual_scales).maxCoeff();
}
RetentionResult ModelRetention::update(const Window & w,const Solver & solve) {
  if(w.times.size()!=static_cast<std::size_t>(w.design.rows()/6) || w.times.empty() ||
     w.design.cols()!=27 || w.wrench.size()!=w.design.rows() || !w.wrench.allFinite() ||
     !std::isfinite(w.start) || !std::isfinite(w.end) || w.end<w.start)
    throw std::invalid_argument("Invalid timestamped retention window");
  for(std::size_t i=0;i<w.times.size();++i)
    if(!std::isfinite(w.times[i]) || (i && w.times[i]<=w.times[i-1]))
      throw std::invalid_argument("Nonmonotonic retention window");
  if(!w.support_starts.empty()) {
    if(w.support_starts.size()!=w.times.size())
      throw std::invalid_argument("Invalid interval support");
    for(std::size_t i=0;i<w.times.size();++i)
      if(!std::isfinite(w.support_starts[i]) || w.support_starts[i]>w.times[i] ||
         (i && w.support_starts[i]<=w.support_starts[i-1]))
        throw std::invalid_argument("Invalid interval support");
  }
  if(!epoch_ || *epoch_!=w.epoch) {
    epoch_=w.epoch;retained_.reset();pending_.reset();stale_=false;
    contradictions_=0;last_validation_end_=w.start;
  }
  RetentionResult out;
  out.information=information(w.design);
  // Validate only on later data, separated from training and from the last
  // validation block. Adjacent rolling fits never count as independent votes.
  auto later=[&](double after)->std::optional<Window> {
    const auto & support=w.support_starts.empty() ? w.times : w.support_starts;
    auto support_first=std::upper_bound(support.begin(),support.end(),after+config_.validation_gap);
    auto first=w.times.begin()+std::distance(support.begin(),support_first);
    if(first==w.times.end() || w.end-*first<config_.validation_seconds)return {};
    const auto index=std::distance(w.times.begin(),first);
    return Window{w.epoch,*support_first,w.end,w.design.bottomRows(w.design.rows()-6*index),
                 w.wrench.tail(w.wrench.size()-6*index),{first,w.times.end()},
                 {support_first,support.end()}};
  };
  if(auto block=later(last_validation_end_)) {
    last_validation_end_=w.end;
    if(retained_) {
      out.retained_error=error(*block,retained_->fit.parameters);
      if(out.retained_error>config_.change_error)++contradictions_;
      else contradictions_=0;
      if(contradictions_>=config_.change_blocks)stale_=true;
    }
  }
  if(pending_) {
    // Accumulate later validation observations until they excite the required
    // combinations; do not throw away useful data every five seconds.
    if(auto block=later(pending_->training.end)) {
      const auto & fresh=*block;
      const auto fresh_info=information(fresh.design);
      out.validation_error=error(fresh,pending_->fit.parameters);
      // Validation must also excite the target combinations. Quiet data must
      // not approve a candidate merely because both models predict rest.
      if(fresh_info.sufficient && fresh_info.net_weight_only==pending_->info.net_weight_only) {
        bool replace=!retained_ || stale_;
        if(retained_ && !stale_) {
          const double retained_error=error(fresh,retained_->fit.parameters);
          const double old_archive_error=error(retained_->training,retained_->fit.parameters);
          const double new_archive_error=error(retained_->training,pending_->fit.parameters);
          const bool improved=out.validation_error<retained_error*(1-config_.improvement_fraction);
          const bool more_info=pending_->info.minimum>=retained_->info.minimum*config_.information_gain &&
            pending_->info.net_weight_only==retained_->info.net_weight_only;
          replace=(improved || more_info) &&
            out.validation_error<=std::max(.01,retained_error*(1+config_.improvement_fraction)) &&
            new_archive_error<=std::max(.01,old_archive_error*(1+config_.improvement_fraction));
        }
        if(out.validation_error<=config_.max_validation_error && replace) {
          retained_=std::move(pending_);retained_->accepted_time=w.end;
          stale_=false;contradictions_=0;out.accepted=true;
        }
        pending_.reset();
      } else if(w.end-pending_->training.end>2*(w.end-w.start)) {
        // Bounded lifetime: don't promote an ancient candidate after a change.
        pending_.reset();
      }
    }
  }
  // Do not refit unobservable parameters from quiet/degenerate windows. Keep
  // monitoring predictions above, and retain the previous validated model.
  // No candidate is published as a successful fit when fitting was skipped.
  if(out.information.sufficient) {
    try {
      auto fit=solve(w.design,w.wrench);
      if(!fit.parameters.allFinite())throw std::runtime_error("Nonfinite candidate");
      out.candidate=fit;
      if(!pending_ && out.information.sufficient)
        pending_=Saved{fit,w,out.information,0};
    }catch(const std::exception & e){out.detail=e.what();}
  }
  out.available=retained_.has_value();
  const bool outside_regime=retained_ && retained_->info.net_weight_only && !out.information.net_weight_only;
  out.valid=out.available && !stale_ && !outside_regime;
  out.contradictions=contradictions_;
  if(retained_) {
    out.retained_net_weight_only=retained_->info.net_weight_only;
    out.parameters=retained_->fit.parameters;out.accepted_time=retained_->accepted_time;
    out.information_at_acceptance=retained_->info.minimum;
    // Continuous monitoring, separate from the disjoint validation votes.
    // Never score training/acceptance data as new measurements for this model.
    const double after=std::max(retained_->accepted_time,w.end-config_.validation_seconds);
    const auto first=std::upper_bound(w.times.begin(),w.times.end(),after);
    if(first!=w.times.end()) {
      const auto index=std::distance(w.times.begin(),first);
      const auto rows=w.design.rows()-6*index;
      out.retained_rms=residual_rms(w.design.bottomRows(rows),w.wrench.tail(rows),out.parameters);
      out.retained_rms_available=true;
      out.retained_rms_samples=w.times.end()-first;
      out.retained_rms_start=*first;out.retained_rms_end=w.end;
    }
  }
  out.status=stale_?"retained_stale_validation_failed":outside_regime?"retained_outside_identified_regime":out.accepted?"retained_updated":
    !out.information.sufficient?(retained_?"holding_retained_insufficient_excitation":"not_identified_insufficient_excitation"):
    pending_?"awaiting_independent_validation":retained_?"holding_retained":"not_identified";
  return out;
}
}  // namespace uvms::identification
