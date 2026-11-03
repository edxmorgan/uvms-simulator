// Exercises read() without configuring transports or publishing actuator commands.
#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/bluerov_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/sim_vehicle_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"
#include <limits>

namespace ros2_control_blue_reach_5 {
struct HardwareEstimatorTestAccess {
  static void run() {
    using casadi::DM;
    BlueRovSystemMultiInterfaceHardware h;
    h.node_topics_interface_ = std::make_shared<rclcpp::Node>("partial_feedback_test");
    const std::string dir = TEST_CASADI_DIR;
    h.estimator_predict_ = casadi::external("disturbance_ekf_predict",dir+"/libdisturbanceEKF_next.so");
    h.estimator_correct_ = casadi::external("disturbance_ekf_correct",dir+"/libdisturbanceEKF_next.so");
    h.utils_service.uv_Exkalman_update = casadi::external("disturbance_ekf_update",dir+"/libdisturbanceEKF_next.so");
    h.utils_service.unwrap = casadi::external("unwrap",dir+"/libAngWrap.so");
    h.utils_service.pwm2rads = casadi::external("pwm_to_rads",dir+"/libPWM_RAD.so");
    h.estimator_nominal_parameters_ = DM(default_nominal_vehicle_parameters());
    h.x_est_=DM::zeros(18,1); h.P_est_=DM::eye(18)*.001;
    h.Q_=DM::eye(18)*.001; h.R_=DM::eye(7)*.01;
    h.hw_vehicle_struct.thrustSizeAllocation(8);
    for(auto &t:h.hw_vehicle_struct.hw_thrust_structs_)t.command_state_.command_pwm=1500;
    auto sensors=[&](){
      h.imu_received_=h.pressure_received_=std::chrono::steady_clock::now();
      h.imu_new_msg_=h.pressure_new_=true;
      h.hw_vehicle_struct.depth_from_pressure2=.1;
      h.hw_vehicle_struct.imu_state.roll=.2;
      h.hw_vehicle_struct.imu_state.pitch=.1;
      h.hw_vehicle_struct.imu_state.yaw=.3;
    };
    sensors();
    h.hw_vehicle_struct.dvl_state.vx=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(h.read(rclcpp::Time(1),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
    EXPECT_TRUE(h.estimator_initialized_);
    EXPECT_TRUE(h.navigation_sensors_valid_);
    EXPECT_EQ(h.estimator_valid_,0.0);
    EXPECT_EQ(h.identification_sample_.valid,0.0);
    EXPECT_NEAR(h.hw_vehicle_struct.estimate_state_.position_z,.1,1e-8);
    EXPECT_NEAR(h.hw_vehicle_struct.estimate_state_.roll,.2,1e-8);
    EXPECT_NEAR(h.hw_vehicle_struct.estimate_state_.pitch,-.1,1e-8);
    // Acquisition of a valid synchronized DVL observation restores the observer.
    sensors();h.use_dvl_=true;h.estimator_dvl_new_=true;
    h.dvl_received_=h.imu_received_;
    h.hw_vehicle_struct.dvl_state.velocity_valid=true;
    h.hw_vehicle_struct.dvl_state.vx=0.;h.hw_vehicle_struct.dvl_state.vy=0.;h.hw_vehicle_struct.dvl_state.vz=0.;
    EXPECT_EQ(h.read(rclcpp::Time(2),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
    EXPECT_EQ(h.estimator_valid_,1.0);
    const auto disturbance=h.x_est_(casadi::Slice(12,18));
    // Bottom-lock loss must not stop pose updates or fabricate a velocity update.
    sensors();h.hw_vehicle_struct.dvl_state.velocity_valid=false;
    h.hw_vehicle_struct.dvl_state.vx=std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(h.read(rclcpp::Time(3),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
    EXPECT_EQ(h.estimator_valid_,0.0);
    EXPECT_TRUE(h.navigation_sensors_valid_);
    EXPECT_EQ(h.identification_sample_.valid,0.0);
    EXPECT_NEAR(double(DM::norm_inf(h.x_est_(casadi::Slice(12,18))-disturbance)),0.,1e-12);
    // DVL recovery is sensor-driven in BOTH manual and non-manual launches.
    for (const bool manual : {false, true}) {
      h.manual_control_=manual;
      sensors(); h.estimator_dvl_new_=true;
      h.dvl_received_=h.imu_received_;
      h.hw_vehicle_struct.dvl_state.velocity_valid=true;
      h.hw_vehicle_struct.dvl_state.vx=.25;
      const double predicted_vx=double(h.estimator_predict_(std::vector<DM>{
          h.x_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.01})[0](6));
      EXPECT_EQ(h.read(rclcpp::Time(4),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
      EXPECT_EQ(h.estimator_valid_,1.0);
      EXPECT_GT(std::abs(double(h.x_est_(6))-predicted_vx),1e-6);
      EXPECT_FALSE(h.estimator_dvl_new_);
      // A stale packet is unavailable even when its validity flag is true.
      sensors(); h.dvl_received_=std::chrono::steady_clock::now()-std::chrono::seconds(1);
      EXPECT_EQ(h.read(rclcpp::Time(5),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
      EXPECT_EQ(h.estimator_valid_,0.0);
      EXPECT_TRUE(h.navigation_sensors_valid_);
    }
    // 70 Hz read loop, 10 Hz IMU/DVL and 2 Hz pressure, with different
    // arrival phases. Receipt ages are injected; no wall-clock sleeps needed.
    for (int step=0; step<350; ++step) {
      const auto now=std::chrono::steady_clock::now();
      h.imu_received_=now-std::chrono::milliseconds((step%7)*14);
      h.pressure_received_=now-std::chrono::milliseconds((step%35)*14+30);
      h.dvl_received_=now-std::chrono::milliseconds(((step+3)%7)*14);
      h.imu_new_msg_=step%7==0;
      h.pressure_new_=step%35==0;
      h.estimator_dvl_new_=(step+3)%7==0;
      h.hw_vehicle_struct.dvl_state.velocity_valid=true;
      EXPECT_EQ(h.read(rclcpp::Time(6),rclcpp::Duration::from_seconds(1./70.)),hardware_interface::return_type::OK);
      EXPECT_TRUE(h.navigation_sensors_valid_);
      EXPECT_EQ(h.estimator_valid_,1.0);
      EXPECT_FALSE(h.imu_new_msg_ || h.pressure_new_ || h.estimator_dvl_new_);
    }
    // Jitter beyond one pressure period does not reset or stop the filter.
    h.pressure_received_=std::chrono::steady_clock::now()-std::chrono::milliseconds(600);
    const auto prediction=h.estimator_predict_(std::vector<DM>{
        h.x_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.01});
    const DM predicted_cov=DM::mtimes(std::vector<DM>{prediction[1],h.P_est_,prediction[1].T()})+h.Q_*.01;
    EXPECT_EQ(h.read(rclcpp::Time(7),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
    EXPECT_TRUE(h.navigation_sensors_valid_);
    EXPECT_EQ(h.estimator_valid_,1.0);
    // No new packets: prediction only, never re-fuse stored measurements.
    EXPECT_NEAR(double(DM::norm_inf(h.x_est_-prediction[0])),0.,1e-10);
    EXPECT_NEAR(double(DM::norm_inf(h.P_est_-predicted_cov)),0.,1e-10);
    h.pressure_received_=std::chrono::steady_clock::now()-std::chrono::seconds(2);
    const double previous_step=h.estimator_step_;
    EXPECT_EQ(h.read(rclcpp::Time(8),rclcpp::Duration::from_seconds(.01)),hardware_interface::return_type::OK);
    EXPECT_FALSE(h.navigation_sensors_valid_);
    EXPECT_EQ(h.estimator_valid_,0.0);
    EXPECT_TRUE(h.estimator_initialized_);
    EXPECT_GT(h.estimator_step_,previous_step);
    sensors();
    h.dvl_received_=h.imu_received_; h.estimator_dvl_new_=true;
    h.read(rclcpp::Time(9),rclcpp::Duration::from_seconds(.01));
    EXPECT_TRUE(h.navigation_sensors_valid_);
    EXPECT_EQ(h.estimator_valid_,1.0);
    // Missing essential sensors must still block manual commands.
    h.imu_received_={};
    h.read(rclcpp::Time(4),rclcpp::Duration::from_seconds(.01));
    EXPECT_FALSE(h.navigation_sensors_valid_);
  }
};
}
TEST(VehiclePartialFeedback, NoDvlAcquisitionAndLoss) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::HardwareEstimatorTestAccess::run();
  rclcpp::shutdown();
}
