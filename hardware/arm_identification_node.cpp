#include "ros2_control_blue_reach_5/arm_identifier.hpp"
#include "ros2_control_blue_reach_5/msg/arm_identification.hpp"
#include <algorithm>
#include <condition_variable>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;
namespace ai = uvms::arm_identification;
using Message = ros2_control_blue_reach_5::msg::ArmIdentification;
class ArmIdentificationNode : public rclcpp::Node {
public:
  ArmIdentificationNode() : Node("arm_identification") {
    joint_ = declare_parameter<std::string>("joint", "robot_1__arm_IOs");
    hardware_ = declare_parameter("hardware_measurements", false);
    enabled_ = declare_parameter("auto_start", true);
    if(hardware_) {
      auto context=declare_parameter<std::vector<double>>("hardware_context",
        {0.,0.,9.81,.19,0.,-.12,3.14159,0.,0.,0.,0.,0.,0.,0.,0.});
      if(context.size()!=15 || !std::all_of(context.begin(),context.end(),[](double v){return std::isfinite(v);}))
        throw std::invalid_argument("hardware_context requires 15 finite gravity/base/world values");
      std::copy(context.begin(),context.end(),hardware_context_.begin());
      RCLCPP_INFO(get_logger(), "Hardware arm identification: observation only; current-derived torque; EKF and controller application disabled");
    }
    support_ = declare_parameter("integration_seconds", 2.);
    gap_ = declare_parameter("max_gap", .15);
    min_sliding_speed_ = declare_parameter("min_sliding_speed", .005);
    arm_config_.horizon = declare_parameter("arm_horizon", 1200);
    arm_config_.residual_scale = declare_parameter("arm_residual_scale", .03);
    arm_config_.min_information = declare_parameter("min_information", 1e-5);
    arm_config_.validation_seconds =
        declare_parameter("validation_seconds", 5.);
    arm_config_.budget = declare_parameter("budget_seconds", 1.);
    gripper_config_ = arm_config_;
    gripper_config_.gripper = true;
    gripper_config_.horizon = declare_parameter("gripper_horizon", 300);
    gripper_config_.residual_scale =
        declare_parameter("gripper_residual_scale", .1);
    // Fail startup on invalid configuration, before creating the worker.
    ai::Window check(support_, gap_, min_sliding_speed_);
    ai::Identifier check_arm(arm_config_), check_gripper(gripper_config_);
    arm_result_ = check_arm.result();
    gripper_result_ = check_gripper.result();
    const auto base = "/arm_identification/" + joint_ + "/";
    arm_pub_ = create_publisher<Message>(base + "arm",
                                         rclcpp::QoS(1).transient_local());
    gripper_pub_ = create_publisher<Message>(base + "gripper",
                                             rclcpp::QoS(1).transient_local());
    summary_pub_ = create_publisher<std_msgs::msg::String>(
        base + "summary", rclcpp::QoS(1).transient_local());
    for (const std::string command : {"start", "stop", "reset"})
      services_.push_back(create_service<std_srvs::srv::Trigger>(
          base + command,
          [this,
           command](const std_srvs::srv::Trigger::Request::SharedPtr,
                    std_srvs::srv::Trigger::Response::SharedPtr response) {
            std::lock_guard<std::mutex> lock(mutex_);
            enabled_ = command == "start";
            reset_ = true;
            ++revision_;
            queue_.clear();
            arm_result_ = ai::Result{};
            gripper_result_ = ai::Result{};
            cv_.notify_one();
            response->success = true;
            response->message = enabled_ ? "Identification started"
                                         : "Identification stopped and "
                                           "cleared; robot motion unchanged";
          }));
    subscription_ = create_subscription<control_msgs::msg::DynamicJointState>(
        "/dynamic_joint_states", rclcpp::SensorDataQoS(),
        [this](control_msgs::msg::DynamicJointState::ConstSharedPtr m) {
          receive(*m);
        });
    timer_ = create_wall_timer(1s, [this]() { publish(); });
    worker_ = std::thread([this]() { work(); });
  }
  ~ArmIdentificationNode() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      exit_ = true;
      cv_.notify_one();
    }
    worker_.join();
  }

private:
  struct Input {
    ai::Sample sample;
    double generation;
  };
  void receive(const control_msgs::msg::DynamicJointState &m) {
    for (size_t j = 0;
         j < m.joint_names.size() && j < m.interface_values.size(); ++j)
      if (m.joint_names[j] == joint_) {
        const auto &v = m.interface_values[j];
        std::unordered_map<std::string, double> fields;
        for (size_t k = 0; k < v.interface_names.size() && k < v.values.size();
             ++k)
          fields[v.interface_names[k]] = v.values[k];
        try {
          Input input;
          input.generation = fields.at("sim_generation");
          auto get = [&](const std::string &name) {
            return fields.at("identification." + name);
          };
          input.sample.time = get("time");
          input.sample.supported = get("supported");
          for (int k = 0; k < 5; ++k) {
            input.sample.position[k] = get("position." + std::to_string(k));
            input.sample.velocity[k] = get("velocity." + std::to_string(k));
            input.sample.impulse[k] = get("impulse." + std::to_string(k));
            input.sample.generation[k] = get("generation." + std::to_string(k));
            input.sample.valid[k] = get("valid." + std::to_string(k));
          }
          for (int k = 0; k < 15; ++k)
            input.sample.context[k] = get("context." + std::to_string(k));
          if(hardware_) input.sample.context=hardware_context_;
          bool finite = std::isfinite(input.generation) &&
                        input.generation >= 0 &&
                        std::isfinite(input.sample.time) &&
                        std::isfinite(input.sample.supported);
          for (double v : input.sample.context)
            finite = finite && std::isfinite(v);
          for (int k = 0; k < 5; ++k)
            finite = finite && std::isfinite(input.sample.position[k]) &&
                     std::isfinite(input.sample.velocity[k]) &&
                     std::isfinite(input.sample.impulse[k]) &&
                     std::isfinite(input.sample.generation[k]) &&
                     std::isfinite(input.sample.valid[k]);
          std::lock_guard<std::mutex> lock(mutex_);
          if (!finite) {
            queue_.clear();
            reset_ = true;
            ++revision_;
            cv_.notify_one();
            return;
          }
          for (int axis = 0; axis < 2; ++axis) {
            const std::string group = axis == 0 ? "arm" : "gripper";
            for (int k = 0; k < 3; ++k) {
              const std::string field =
                  "estimator_identification." + group + "." +
                  std::array<std::string, 3>{"active", "epoch",
                                             "accepted_time"}[k];
              auto value = fields.find(field);
              acknowledgement_[3 * axis + k] =
                  value == fields.end() ? -1 : value->second;
            }
          }
          if (!enabled_)
            return;
          if (queue_.size() >= 500) {
            queue_.clear();
            reset_ = true;
            ++revision_;
          }
          queue_.push_back(input);
          cv_.notify_one();
        } catch (const std::out_of_range &) { /* Older/hardware plugins do not
                                                 expose this contract. */
        }
        return;
      }
  }
  void work() {
    ai::Window window(support_, gap_, min_sliding_speed_);
    ai::Identifier arm(arm_config_), gripper(gripper_config_);
    double generation = -1, last_time = -1;
    std::array<double, 15> context{};
    uint64_t epoch = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    while (true) {
      Input input;
      bool have = false, reset = false;
      uint64_t revision;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&]() { return exit_ || reset_ || !queue_.empty(); });
        if (exit_)
          return;
        revision = revision_;
        reset = reset_;
        reset_ = false;
        if (!queue_.empty()) {
          input = queue_.front();
          queue_.pop_front();
          have = true;
        }
      }
      if (reset) {
        ++epoch;
        window.reset();
        arm.reset();
        gripper.reset();
        last_time = -1;
      }
      if (have) {
        auto &s = input.sample;
        if (input.generation != generation || s.time < last_time ||
            s.context != context) {
          ++epoch;
          window.reset();
          arm.configure(s);
          gripper.configure(s);
          context = s.context;
          generation = input.generation;
          last_time = -1;
        }
        if (s.time != last_time) {
          if (last_time >= 0 && s.time - last_time > gap_) {
            ++epoch;
            window.reset();
            arm.reset();
            gripper.reset();
          }
          if (!s.supported) {
            ++epoch;
            window.reset();
            arm.reset();
            gripper.reset();
          } else {
            auto rows = window.observe(s);
            arm.observe(rows);
            gripper.observe(rows);
          }
          last_time = s.time;
          std::lock_guard<std::mutex> lock(mutex_);
          last_measurement_ = std::chrono::steady_clock::now();
          supported_ = s.supported;
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (revision == revision_) {
          arm_result_ = arm.result();
          gripper_result_ = gripper.result();
          source_generation_ = std::max(0., generation);
          epoch_ = epoch;
          observation_time_ = last_time;
          model_context_ = context;
        }
      }
    }
  }
  void publish() {
    std::lock_guard<std::mutex> lock(mutex_);
    bool fresh = std::chrono::steady_clock::now() - last_measurement_ < 1s;
    auto message = [&](const ai::Result &r, const std::string &subsystem) {
      Message m;
      m.stamp = now();
      m.joint = joint_;
      m.subsystem = subsystem;
      m.source_generation = source_generation_;
      m.measurements_fresh = fresh;
      m.retained_valid = r.valid;
      m.epoch = epoch_;
      m.observation_time = observation_time_;
      m.model_context = model_context_;
      m.retained_accepted_time = r.accepted_time;
      const int ack = subsystem == "arm" ? 0 : 3;
      m.applied = !hardware_ && r.valid && acknowledgement_[ack] == 1 &&
                  acknowledgement_[ack + 1] == double(epoch_) &&
                  acknowledgement_[ack + 2] == r.accepted_time;
      m.status =
          !enabled_ ? "stopped"
          : !fresh  ? "waiting for fresh measurements"
          : !supported_
              ? "requires uncoupled arm, zero payload and no Stribeck friction"
              : r.status;
      if(hardware_ && subsystem=="gripper") m.status="hardware gripper identification not qualified";
      m.detail = r.detail;
      m.rows = r.rows;
      m.information_rank = r.rank;
      m.information_dimensions = r.dimensions;
      m.min_information = r.min_information;
      if (r.candidate.size())
        m.candidate_parameters.assign(r.candidate.data(),
                                      r.candidate.data() + r.candidate.size());
      if (r.retained.size())
        m.retained_parameters.assign(r.retained.data(),
                                     r.retained.data() + r.retained.size());
      m.validation_normalized_rms = r.validation_error;
      m.retained_normalized_rms = r.retained_error;
      return m;
    };
    auto arm = message(arm_result_, "arm"),
         gripper = message(gripper_result_, "gripper");
    arm_pub_->publish(arm);
    gripper_pub_->publish(gripper);
    std_msgs::msg::String summary;
    auto label = [this](const Message &m) {
      const bool arm = m.subsystem == "arm";
      const auto &result = arm ? arm_result_ : gripper_result_;
      const int required = arm ? arm_config_.horizon : gripper_config_.horizon;
      std::string collection;
      if (arm && m.status == "collecting") {
        collection = "; accepted rows [base,shoulder,elbow,wrist]=";
        for (int axis=0; axis<4; ++axis)
          collection += (axis ? "," : "") + std::to_string(result.rows_by_axis[axis]);
        collection += "; each usable interval requires " + std::to_string(support_) +
          " s of valid feedback with speed >= " + std::to_string(min_sliding_speed_) +
          " rad/s; pauses, reversals and limit contact exclude intervals";
      }
      return m.subsystem + ": " + m.status +
             "; rows=" + std::to_string(m.rows) + "/" + std::to_string(required) + collection +
             (m.status == "collecting" ? "; rank=not assessed yet" :
              "; rank=" + std::to_string(m.information_rank) + "/" +
              std::to_string(m.information_dimensions)) +
             "; model=" + (m.retained_valid ? "valid" : "not validated") +
             (hardware_ ? "; observation only; EKF unchanged"
              : m.applied          ? "; EKF: identified model applied"
              : m.retained_valid ? "; EKF: new model pending acknowledgement"
                                 : "; EKF: awaiting validated model") +
             (m.detail.empty() ? "" : "; " + m.detail);
    };
    summary.data = label(arm) + " | " + label(gripper);
    summary_pub_->publish(summary);
  }
  bool hardware_ = false;
  std::array<double,15> hardware_context_{};
  std::string joint_;
  double support_, gap_, min_sliding_speed_;
  ai::Config arm_config_, gripper_config_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Input> queue_;
  std::thread worker_;
  bool exit_ = false, reset_ = false, enabled_ = true, supported_ = false;
  uint64_t source_generation_ = 0, revision_ = 0, epoch_ = 0;
  double observation_time_ = -1;
  std::array<double, 15> model_context_{};
  std::array<double, 6> acknowledgement_{0, -1, -1, 0, -1, -1};
  std::chrono::steady_clock::time_point last_measurement_{};
  ai::Result arm_result_, gripper_result_;
  rclcpp::Publisher<Message>::SharedPtr arm_pub_, gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ArmIdentificationNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("arm_identification"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
