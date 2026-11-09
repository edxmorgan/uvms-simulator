#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include "ros2_control_blue_reach_5/msg/vehicle_identification.hpp"
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <future>
#include <iostream>
#include <map>
#include <limits>

using namespace std::chrono_literals;
namespace vi = uvms::identification;
using Diagnostic = ros2_control_blue_reach_5::msg::VehicleIdentification;
class VehicleIdentificationNode : public rclcpp::Node {
 public:
  VehicleIdentificationNode():Node("vehicle_identification") {
    rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
    hardware_=declare_parameter("hardware_measurements",false,fixed);
    confirmed_=hardware_?
      declare_parameter("confirm_hardware_thrust_calibration_zero_current_uncoupled",false,fixed):
      declare_parameter("confirm_sim_effort_zero_current_uncoupled",false,fixed);
    joint_=declare_parameter<std::string>("joint","robot_1_IOs",fixed);
    auto bounds=declare_parameter<std::vector<double>>("weight_bounds",{},fixed);
    if(bounds.size()!=2)throw std::invalid_argument("Configure independent weight_bounds [min,max]");
    lower_=bounds[0];upper_=bounds[1];
    budget_=declare_parameter("budget_seconds",.5,fixed);
    gap_=declare_parameter("max_gap",.05,fixed);
    period_=declare_parameter("fit_period",1.,fixed);
    int horizon=declare_parameter("horizon",4200,fixed);
    if(horizon<5 || horizon>100000 || !std::isfinite(period_) || period_<=0)
      throw std::invalid_argument("Invalid horizon/fit period");
    window_=std::make_unique<vi::LiveWindow>(horizon,gap_);
    // Instantiate/check configuration here; all solves run exclusively on the worker.
    identifier_=std::make_unique<vi::Identifier>(lower_,upper_,budget_);
    vi::RetentionConfig retention_config;
    auto scales=declare_parameter<std::vector<double>>("retention_parameter_scales",
      std::vector<double>(retention_config.parameter_scales.data(),retention_config.parameter_scales.data()+27),fixed);
    auto residual_scales=declare_parameter<std::vector<double>>("retention_residual_scales",
      std::vector<double>(retention_config.residual_scales.data(),retention_config.residual_scales.data()+6),fixed);
    if(scales.size()!=27 || residual_scales.size()!=6)
      throw std::invalid_argument("Retention requires 27 parameter scales and 6 residual scales");
    std::copy(scales.begin(),scales.end(),retention_config.parameter_scales.data());
    std::copy(residual_scales.begin(),residual_scales.end(),retention_config.residual_scales.data());
    retention_config.min_information=declare_parameter("retention_min_information",retention_config.min_information,fixed);
    retention_config.validation_seconds=declare_parameter("retention_validation_seconds",retention_config.validation_seconds,fixed);
    retention_config.validation_gap=declare_parameter("retention_validation_gap",retention_config.validation_gap,fixed);
    retention_config.max_validation_error=declare_parameter("retention_max_validation_error",retention_config.max_validation_error,fixed);
    retention_config.change_error=declare_parameter("retention_change_error",retention_config.change_error,fixed);
    retention_config.change_blocks=declare_parameter("retention_change_blocks",retention_config.change_blocks,fixed);
    retention_=std::make_unique<vi::ModelRetention>(retention_config,lower_==upper_);
    auto_start_=declare_parameter("auto_start",false,fixed);
    publisher_=create_publisher<Diagnostic>("/vehicle_identification/"+joint_+"/candidate",10);
    const auto base="/vehicle_identification/"+joint_+"/";
    status_=confirmed_?"checking_solver":"awaiting_configuration_confirmation";
    for(const std::string command:{"start","stop","reset","status","confirm_configuration"}) {
      services_.push_back(create_service<std_srvs::srv::Trigger>(base+command,
        [this,command](const std_srvs::srv::Trigger::Request::SharedPtr,
                       std_srvs::srv::Trigger::Response::SharedPtr response){
          if(command=="confirm_configuration") {
            if(!confirmed_) {
              confirmed_=true;status_="checking_solver";
              preflight_=std::async(std::launch::async,[this](){identifier_->preflight();});
            }
          } else if(command=="start") {
            if(!solver_ready_) {response->success=false;response->message=status_+": "+detail_;return;}
            if(!enabled_) {window_->reset();enabled_=true;status_="waiting_for_valid_window";detail_.clear();}
          } else if(command=="stop" || command=="reset") {
            // Epoch invalidation prevents an in-flight worker from publishing an old candidate.
            auto_start_=false;enabled_=false;window_->reset();
            if(solver_ready_) {status_="idle";detail_.clear();}
          }
          response->success=true;
          response->message=status();
          auto m=message(status_);m.detail=detail_;publisher_->publish(m);
        }));
    }
    if(confirmed_)preflight_=std::async(std::launch::async,[this](){identifier_->preflight();});
    subscription_=create_subscription<control_msgs::msg::DynamicJointState>(
      "/dynamic_joint_states",rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::DynamicJointState::ConstSharedPtr message){receive(*message);});
    timer_=create_wall_timer(20ms,[this](){tick();});
    RCLCPP_INFO(get_logger(),"Native MOSEK estimator: %s; retained models feed simulation EKF only",status_.c_str());
  }
  ~VehicleIdentificationNode() override {
    if(preflight_.valid())preflight_.wait();
    if(future_.valid())future_.wait();
  }
 private:
  using Clock=std::chrono::steady_clock;
  std::string status() const {
    return status_+"; enabled="+(enabled_?"true":"false")+
      "; weight_bounds=["+std::to_string(lower_)+", "+std::to_string(upper_)+"] N; confirmed="+(confirmed_?"true":"false")+
      "; solver_ready="+(solver_ready_?"true":"false")+
      "; samples="+std::to_string(window_->size())+"; simulation_ekf_applied="+(applied()?"true":"false")+"; "+detail_;
  }
  bool applied() const {
    return !hardware_ && retention_epoch_==window_->epoch() && last_retention_.available &&
      applied_retained_==1. && applied_epoch_==retention_epoch_ &&
      applied_time_==last_retention_.accepted_time;
  }
  double age(Clock::time_point t) const {return std::chrono::duration<double>(Clock::now()-t).count();}
  Diagnostic message(const std::string & status) {
    Diagnostic m;m.stamp=now();m.joint=joint_;m.status=status;m.applied=applied();
    m.source_generation=source_generation_;m.hardware_measurements=hardware_;
    m.epoch=window_->epoch();m.window_samples=window_->size();m.weight_bounds={lower_,upper_};
    m.validation_error=m.retained_validation_error=-1;
    if(retention_epoch_==m.epoch) {
      const auto & r=last_retention_;
      m.retained_available=r.available;
      m.retained_valid=r.valid && enabled_ && age(last_received_)<=gap_;
      m.retained_net_weight_only=r.retained_net_weight_only;
      std::copy(r.parameters.data(),r.parameters.data()+27,m.retained_parameters.begin());
      m.retained_accepted_time=r.accepted_time;
      m.retained_min_information=r.information_at_acceptance;
      m.information_rank=r.information.rank;m.information_dimensions=r.information.dimensions;
      m.min_information=r.information.minimum;m.information_condition=r.information.condition;
      m.net_weight_only=r.information.net_weight_only;
      m.validation_error=r.validation_error;m.retained_validation_error=r.retained_error;
      m.contradiction_blocks=r.contradictions;
      m.retained_rms_available=r.retained_rms_available && enabled_ && age(last_received_)<=gap_;
      std::copy(r.retained_rms.data(),r.retained_rms.data()+6,m.retained_residual_rms.begin());
      m.retained_rms_samples=r.retained_rms_samples;
      m.retained_rms_window_start=r.retained_rms_start;
      m.retained_rms_window_end=r.retained_rms_end;
    }
    return m;
  }
  void receive(const control_msgs::msg::DynamicJointState & msg) {
    if(!enabled_)return;
    try {
      auto it=std::find(msg.joint_names.begin(),msg.joint_names.end(),joint_);
      if(it==msg.joint_names.end())throw std::invalid_argument("Configured joint missing");
      auto i=std::distance(msg.joint_names.begin(),it);
      if(static_cast<std::size_t>(i)>=msg.interface_values.size())throw std::invalid_argument("Malformed state");
      const auto & values=msg.interface_values[i];
      if(values.interface_names.size()!=values.values.size())throw std::invalid_argument("Malformed interfaces");
      std::map<std::string,double> v;
      for(std::size_t j=0;j<values.values.size();++j)
        if(!v.emplace(values.interface_names[j],values.values[j]).second)throw std::invalid_argument("Duplicate interface");
      vi::Measurement m;
      if(hardware_) {
        if(v.at("identification.valid")!=1.) {
          if(window_->size())window_->reset();
          last_received_={};
          return;
        }
        m.generation=v.at("identification.generation");m.step=v.at("identification.step");
        m.time=v.at("identification.time");m.period=v.at("identification.period");
        m.depth=v.at("identification.depth");
        for(int j=0;j<3;++j)m.attitude[j]=v.at("identification.attitude."+std::to_string(j));
        for(int j=0;j<6;++j) {
          m.velocity[j]=v.at("identification.velocity."+std::to_string(j));
          m.wrench[j]=v.at("identification.wrench."+std::to_string(j));
        }
      } else {
      // This interval regressor requires adjacent, fresh physics-step endpoints.
      // Multirate held values are not independent observations. Do not silently
      // fit them; an asynchronous impulse/endpoints adapter is a separate task.
      if (v.count("sensor_new.0")) {
        bool fresh=true;
        for (int channel=0; channel<7; ++channel)
          fresh = fresh && v.at("sensor_new."+std::to_string(channel))==1.0;
        if (!fresh) {
          window_->reset();last_received_={};
          detail_="Multirate sensor observations: identifier requires fresh adjacent-step endpoints; no fit performed";
          return;
        }
      }
      m.generation=v.at("sim_generation");m.step=v.at("sim_step");m.time=v.at("sim_time");
      applied_retained_=v.at("estimator_model_retained");
      applied_epoch_=v.at("estimator_model_epoch");
      applied_time_=v.at("estimator_model_accepted_time");
      m.period=v.at("sim_period");m.depth=v.at("depth_from_pressure2");
      m.attitude<<v.at("imu_roll"),-v.at("imu_pitch"),-v.at("imu_yaw");
      m.velocity<<v.at("dvl_speed_x"),v.at("dvl_speed_y"),v.at("dvl_speed_z"),
        v.at("imu_angular_vel_x"),-v.at("imu_angular_vel_y"),-v.at("imu_angular_vel_z");
      m.wrench<<v.at("force.x"),v.at("force.y"),v.at("force.z"),v.at("torque.x"),v.at("torque.y"),v.at("torque.z");
      }
      if(window_->push(m)) {
        source_generation_=static_cast<uint64_t>(m.generation);
        last_received_=Clock::now();
      }
    }catch(const std::exception & e){
      window_->reset();
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Invalid identification measurements: %s",e.what());
    }
  }
  void tick(){
    if(preflight_.valid()) {
      if(preflight_.wait_for(0s)!=std::future_status::ready)return;
      try {preflight_.get();solver_ready_=true;enabled_=auto_start_;status_=enabled_?"waiting_for_valid_window":"idle";}
      catch(const std::exception & e) {status_="solver_unavailable";detail_=e.what();}
      if(solver_ready_)
        RCLCPP_INFO(get_logger(),"MOSEK ready; background observation %s; controller and hardware model application disabled",enabled_?"enabled":"disabled by configuration");
      else
        RCLCPP_WARN(get_logger(),"Background identification unavailable: %s. Operation can continue without identification.",detail_.c_str());
      auto m=message(status_);m.detail=detail_;publisher_->publish(m);
    }
    if(future_.valid()){
      if(future_.wait_for(0s)!=std::future_status::ready)return;
      auto m=message("solve_rejected");m.epoch=inflight_epoch_;
      m.window_start=inflight_start_;m.window_end=inflight_end_;
      try{
        auto result=future_.get();
        if(window_->epoch()!=inflight_epoch_ || age(last_received_)>gap_)m.status="discarded_stale_candidate";
        else{
          last_retention_=result;retention_epoch_=inflight_epoch_;
          m=message(result.status);m.detail=result.detail;
          m.window_start=inflight_start_;m.window_end=inflight_end_;
          if(result.candidate) {
          const auto & fit=*result.candidate;
          std::copy(fit.parameters.data(),fit.parameters.data()+27,m.parameters.begin());
          std::copy(fit.residual_rms.data(),fit.residual_rms.data()+6,m.residual_rms.begin());
          m.numerical_rank=fit.rank;m.robust_iterations=fit.iterations;
          m.relative_objective_change=fit.relative_change;m.solver_seconds=fit.solver_seconds;m.fit_seconds=fit.wall_seconds;
          }
        }
      }catch(const std::exception & e){m.detail=e.what();}
      if(enabled_ && window_->epoch()==inflight_epoch_) {
        status_=m.status;detail_=m.detail;publisher_->publish(m);
      }
    }
    if(age(last_attempt_)<period_)return;
    last_attempt_=Clock::now();
    if(!enabled_) {auto m=message(status_);m.detail=detail_;publisher_->publish(m);return;}
    auto w=window_->snapshot();
    if(!w || age(last_received_)>gap_){status_="waiting_for_valid_window";publisher_->publish(message(status_));return;}
    inflight_epoch_=w->epoch;inflight_start_=w->start;inflight_end_=w->end;
    future_=std::async(std::launch::async,[this,w=std::move(*w)](){
      return retention_->update(w,[this](const auto & a,const auto & b){return identifier_->fit(a,b);});
    });
  }
  std::string joint_;
  uint64_t source_generation_=0;
  double applied_retained_=0.,applied_epoch_=-1.,applied_time_=-1.;
  double lower_,upper_,budget_,gap_,period_,inflight_start_=0,inflight_end_=0;
  std::size_t inflight_epoch_=0;
  std::unique_ptr<vi::LiveWindow> window_;
  std::unique_ptr<vi::Identifier> identifier_;
  std::unique_ptr<vi::ModelRetention> retention_;
  vi::RetentionResult last_retention_;
  std::size_t retention_epoch_=std::numeric_limits<std::size_t>::max();
  std::future<vi::RetentionResult> future_;
  std::future<void> preflight_;
  bool enabled_=false,solver_ready_=false,auto_start_=false,hardware_=false;
  bool confirmed_=false;
  std::string status_="checking_solver",detail_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
  Clock::time_point last_received_{},last_attempt_{};
  rclcpp::Publisher<Diagnostic>::SharedPtr publisher_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char ** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<VehicleIdentificationNode>());}
  catch(const std::exception & e){std::cerr<<"vehicle_identification: "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
