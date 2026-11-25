#include "ros2_control_blue_reach_5/reach_system_multi_Interface.hpp"
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

namespace ros2_control_blue_reach_5 {
struct ReachLifecycleTestAccess {
  using Hardware = ReachSystemMultiInterfaceHardware;
  static void start(Hardware &h, const char *port) {
    h.driver_.start(port,1,false);
    h.driver_started_=true;
    h.hw_joint_struct_.resize(5);
    h.claimed_modes_.assign(5,"effort");
    h.control_level_.assign(5,Hardware::mode_level_t::MODE_EFFORT);
    h.feedback_clocks_.resize(5);
    for(auto &joint:h.hw_joint_struct_) {
      joint.command_state_.effort=2.;
      joint.command_state_.current=1.;
      joint.command_state_.velocity=.5;
    }
    for(auto &clock:h.feedback_clocks_)
      for(unsigned field=0;field<3;++field)
        clock.update(field,JointFeedbackClock::Clock::now());
    h.command_fault_=true;
    h.running_=true;
    h.state_request_worker_=std::thread([&h] {h.pollState(20);});
  }
  static void check_stale_feedback_does_not_latch(Hardware &h) {
    h.command_fault_ = false;
    // Reproduce a selective reply outage: devices 1/2 stale, others fresh.
    {
      const std::lock_guard<std::mutex> lock(h.access_async_states_);
      for(unsigned joint=0; joint<2; ++joint)
        for(unsigned field=0; field<3; ++field)
          h.feedback_clocks_[joint].update(field,
            JointFeedbackClock::Clock::now()-std::chrono::seconds(1));
    }
    // Exercise the real write callback without sending movement commands.
    h.serial_budget_.command_due(SerialBudget::Clock::now());
    EXPECT_EQ(h.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),
      hardware_interface::return_type::OK);
    EXPECT_FALSE(h.command_fault_);
    EXPECT_TRUE(h.driver_started_);
    EXPECT_TRUE(h.running_.load());
    for(auto &mode:h.claimed_modes_) EXPECT_EQ(mode,"effort");
    EXPECT_TRUE(h.feedback_dropout_);
    const auto detection = h.feedback_dropout_start_;
    h.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
    EXPECT_EQ(h.feedback_dropout_start_, detection);
    {
      const std::lock_guard<std::mutex> lock(h.access_async_states_);
      for (auto &clock : h.feedback_clocks_)
        for (unsigned field=0; field<3; ++field)
          clock.update(field, JointFeedbackClock::Clock::now());
    }
    EXPECT_EQ(h.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),
      hardware_interface::return_type::OK);
    EXPECT_FALSE(h.feedback_dropout_);
    // Preserve the explicit transport-fault case for the lifecycle test.
    h.command_fault_ = true;
  }
  static void check_stopped(Hardware &h) {
    EXPECT_FALSE(h.running_.load());
    EXPECT_FALSE(h.driver_started_);
    EXPECT_FALSE(h.state_request_worker_.joinable());
    EXPECT_FALSE(h.arm_observer_worker_);
    EXPECT_FALSE(h.spin_thread_.joinable());
    // A latched fault is only cleared by explicit activation.
    EXPECT_TRUE(h.command_fault_);
    for(auto &clock:h.feedback_clocks_)
      EXPECT_FALSE(clock.fresh(JointFeedbackClock::Clock::now()));
    for(auto &mode:h.claimed_modes_) EXPECT_TRUE(mode.empty());
    for(auto &joint:h.hw_joint_struct_) {
      EXPECT_EQ(joint.command_state_.effort,0.);
      EXPECT_EQ(joint.command_state_.current,0.);
      EXPECT_EQ(joint.command_state_.velocity,0.);
    }
    EXPECT_THROW(h.driver_.request(alpha::driver::PacketId::PacketID_POSITION,
      alpha::driver::DeviceId::kRotateBase),std::runtime_error);
  }
};

TEST(ReachLifecycle, FaultAndShutdownStopWorkersAndPermitSerialReconfiguration) {
  int master=-1,slave=-1; char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  {
    ReachSystemMultiInterfaceHardware hardware;
    ReachLifecycleTestAccess::start(hardware,name);
    ReachLifecycleTestAccess::check_stale_feedback_does_not_latch(hardware);
    EXPECT_EQ(hardware.on_error(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    ReachLifecycleTestAccess::check_stopped(hardware);
    EXPECT_EQ(hardware.on_cleanup(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    // Reopening the same driver exercises joining and closing, not just flags.
    ReachLifecycleTestAccess::start(hardware,name);
    EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
    ReachLifecycleTestAccess::check_stopped(hardware);
    EXPECT_EQ(hardware.on_shutdown(rclcpp_lifecycle::State()),
      hardware_interface::CallbackReturn::SUCCESS);
  }
  close(master);close(slave);
}
}
