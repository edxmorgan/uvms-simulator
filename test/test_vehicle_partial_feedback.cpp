// Exercises read() without configuring transports or publishing actuator commands.
#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/bluerov_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/sim_vehicle_system_multi_interface.hpp"
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"
#include <limits>

namespace ros2_control_blue_reach_5 {
struct HardwareEstimatorTestAccess {
  static void adaptation_configuration() {
    BlueRovSystemMultiInterfaceHardware h;
    const std::vector<double> full{.001,0.,0.,0.,0.,0., 0.,.001,0.,0.,0.,0.,
      0.,0.,.001,0.,0.,0., 0.,0.,0.,.001,0.,0., 0.,0.,0.,0.,.001,0., 0.,0.,0.,0.,0.,.001};
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("adaptation_full_covariance_test",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("ekf_adapt_wrench_process_noise",true),
        rclcpp::Parameter("ekf_wrench_process_covariance",full)}));
    EXPECT_THROW(h.configure_sensor_measurements(),std::invalid_argument);
  }
  static void covariance_and_pressure() {
    using casadi::DM;
    BlueRovSystemMultiInterfaceHardware h;
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("hardware_sensor_config_test",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("surface_pressure_pa",100000.)}));
    h.configure_sensor_measurements();
    sensor_msgs::msg::Imu imu;imu.header.frame_id="base_link";
    imu.header.stamp=rclcpp::Time(1,0,RCL_ROS_TIME);
    imu.angular_velocity.x=.1;imu.angular_velocity.y=.2;imu.angular_velocity.z=.3;
    imu.linear_acceleration.z=9.80665;
    h.receive_raw_imu(imu);
    EXPECT_DOUBLE_EQ(h.raw_imu_measurement_[1],-.2);
    EXPECT_DOUBLE_EQ(h.raw_imu_measurement_[5],-9.80665);
    EXPECT_DOUBLE_EQ(h.raw_imu_new_[0],1.);
    EXPECT_TRUE(h.raw_imu_available_[0]); EXPECT_TRUE(h.raw_imu_available_[1]);
    h.raw_imu_new_.fill(0.);h.receive_raw_imu(imu);
    EXPECT_DOUBLE_EQ(h.raw_imu_new_[0],0.); // identical timestamp is never new
    imu.header.stamp=rclcpp::Time(2,0,RCL_ROS_TIME);
    imu.linear_acceleration_covariance[0]=-1.;h.receive_raw_imu(imu);
    EXPECT_DOUBLE_EQ(h.raw_imu_new_[0],1.);EXPECT_DOUBLE_EQ(h.raw_imu_new_[3],0.);
    EXPECT_TRUE(h.raw_imu_available_[0]); EXPECT_FALSE(h.raw_imu_available_[1]);
    EXPECT_DOUBLE_EQ(h.pressureToDepth(1000.,997.),0.);
    EXPECT_NEAR(h.pressureToDepth((100000.+997.*9.80665*2.)/100.,997.),2.,1e-12);
    EXPECT_THROW(h.pressureToDepth(1000.,0.),std::invalid_argument);
    h.R_=DM::eye(7);
    const std::array<double,9> covariance{.0001,.00002,-.00001,.00002,.0002,.00003,-.00001,.00003,.0003};
    h.hw_vehicle_struct.dvl_state.covariance.data=covariance;
    ASSERT_TRUE(h.update_dvl_measurement_covariance());
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)
      EXPECT_DOUBLE_EQ(double(h.R_(4+i,4+j)),covariance[3*i+j]);
    const auto good=h.R_;
    for (auto bad:std::vector<std::array<double,9>>{
        {},{-1.,0.,0.,0.,1.,0.,0.,0.,1.},
        {1.,2.,0.,2.,1.,0.,0.,0.,1.},
        {1.,.1,0.,0.,1.,0.,0.,0.,1.},
        {std::numeric_limits<double>::quiet_NaN(),0.,0.,0.,1.,0.,0.,0.,1.}}) {
      h.hw_vehicle_struct.dvl_state.covariance.data=bad;
      EXPECT_FALSE(h.update_dvl_measurement_covariance());
      EXPECT_DOUBLE_EQ(double(DM::norm_inf(h.R_-good)),0.);
    }
  }
  static void run() {
    using casadi::DM;
    BlueRovSystemMultiInterfaceHardware h;
    h.node_topics_interface_ = std::make_shared<rclcpp::Node>("partial_feedback_test");
    const std::string dir = TEST_CASADI_DIR;
    h.estimator_predict_ = casadi::external("disturbance_ekf_predict",dir+"/libdisturbanceEKF_next.so");
    h.estimator_covariance_predict_=casadi::external("disturbance_ekf_predict_covariance",dir+"/libdisturbanceEKF_next.so");
    h.estimator_correct_ = casadi::external("disturbance_ekf_correct",dir+"/libdisturbanceEKF_next.so");
    h.sensor_correct_ = casadi::external("disturbance_ekf_sensor_correct",dir+"/libdisturbanceEKF_next.so");
    h.sensor_model_=casadi::external("vehicle_sensor_model",dir+"/libdisturbanceEKF_next.so");
    h.utils_service.uv_Exkalman_update = casadi::external("disturbance_ekf_update",dir+"/libdisturbanceEKF_next.so");
    h.utils_service.unwrap = casadi::external("unwrap",dir+"/libAngWrap.so");
    h.utils_service.pwm2rads = casadi::external("pwm_to_rads",dir+"/libPWM_RAD.so");
    h.estimator_nominal_parameters_ = DM(default_nominal_vehicle_parameters());
    h.x_est_=DM::zeros(18,1); h.P_est_=DM::eye(18)*.001;
    h.Q_=DM::eye(18)*.001; h.R_=DM::eye(7)*.01;
    h.hw_vehicle_struct.dvl_state.covariance.data={.0001,.00002,0.,.00002,.0002,0.,0.,0.,.0003};
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
    const auto continuous_prediction=casadi_reach_alpha_5::predict_disturbance_covariance(
        h.estimator_covariance_predict_,h.x_est_,h.P_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.01,h.Q_);
    const DM predicted_cov=continuous_prediction[1];
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

namespace ros2_control_blue_reach_5 {
struct SimEstimatorTestAccess {
  static void independent_measurement_noise() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware baseline, assumed;
    baseline.node_topics_interface_=std::make_shared<rclcpp::Node>("noise_baseline");
    assumed.node_topics_interface_=std::make_shared<rclcpp::Node>("noise_assumed",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("depth_attitude_measurement_variances",std::vector<double>{.01,.02,.03,.04}),
        rclcpp::Parameter("dvl_measurement_covariance",std::vector<double>{.1,.01,0.,.01,.2,0.,0.,0.,.3}),
        rclcpp::Parameter("raw_imu_measurement_variances",std::vector<double>{.1,.2,.3,.4,.5,.6})}));
    baseline.configure_estimator_noise();assumed.configure_estimator_noise();
    EXPECT_DOUBLE_EQ(baseline.sensor_noise_enabled_,1.);
    EXPECT_DOUBLE_EQ(baseline.apply_sensor_noise(.123),.123);
    baseline.sensor_noise_requested_.store(false);
    assumed.sensor_noise_requested_.store(false);
    EXPECT_EQ(baseline.sample_vehicle_measurements(std::vector<double>(12,0.)),
              (std::array<double,7>{}));
    assumed.sample_vehicle_measurements(std::vector<double>(12,0.));
    baseline.sensor_noise_requested_.store(true);
    assumed.sensor_noise_requested_.store(true);
    for(int i=0;i<4;++i) EXPECT_DOUBLE_EQ(double(assumed.R_(i,i)),.01*(i+1));
    EXPECT_DOUBLE_EQ(double(assumed.R_(4,5)),.01);
    EXPECT_DOUBLE_EQ(double(assumed.sensor_R_(12,12)),.6);
    EXPECT_EQ(baseline.raw_imu_std_,assumed.raw_imu_std_);
    for(int i=0;i<100;++i)
      EXPECT_EQ(baseline.sample_vehicle_measurements(std::vector<double>(12,0.)),
                assumed.sample_vehicle_measurements(std::vector<double>(12,0.)));
    SimVehicleSystemMultiInterfaceHardware noisy;
    noisy.node_topics_interface_=std::make_shared<rclcpp::Node>("noise_changed",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sim_sensor_noise_std",std::vector<double>(7,.2)),
        rclcpp::Parameter("sim_raw_imu_noise_std",std::vector<double>(6,.3))}));
    noisy.configure_estimator_noise();
    EXPECT_DOUBLE_EQ(double(DM::norm_inf(noisy.sensor_R_-baseline.sensor_R_)),0.);
    for(const auto& name: {"depth_attitude_measurement_variances", "raw_imu_measurement_variances", "dvl_measurement_covariance"}) {
      SimVehicleSystemMultiInterfaceHardware invalid;
      invalid.node_topics_interface_=std::make_shared<rclcpp::Node>("invalid_assumed_noise",
        rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter(name,std::vector<double>{-1.})}));
      EXPECT_THROW(invalid.configure_estimator_noise(),std::invalid_argument);
    }
  }

  static void correlated_dvl() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware h;
    const std::vector<double> covariance{.0001,.00004,-.00002,.00004,.0002,.00003,-.00002,.00003,.0003};
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("correlated_dvl_test",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sim_sensor_rates_hz",std::vector<double>{0.,0.,0.}),
        rclcpp::Parameter("sim_sensor_noise_enabled",true),
        rclcpp::Parameter("sim_dvl_covariance",covariance)}));
    h.configure_estimator_noise();
    std::array<double,9> empirical{};
    for(int n=0;n<100000;++n) {
      const auto y=h.sample_vehicle_measurements(std::vector<double>(12,0.));
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)empirical[3*i+j]+=y[4+i]*y[4+j];
    }
    for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
      EXPECT_DOUBLE_EQ(double(h.R_(4+i,4+j)),i==j ? 1e-6 : 0.);
      EXPECT_NEAR(empirical[3*i+j]/100000.,covariance[3*i+j],
        .02*std::sqrt(covariance[3*i+i]*covariance[3*j+j]));
    }
    SimVehicleSystemMultiInterfaceHardware invalid;
    invalid.node_topics_interface_=std::make_shared<rclcpp::Node>("invalid_dvl_covariance_test",
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("sim_dvl_covariance",std::vector<double>(9,0.))}));
    EXPECT_THROW(invalid.configure_estimator_noise(),std::invalid_argument);
  }
  static void multirate() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware h;
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("multirate_sensor_test",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sim_sensor_rates_hz",std::vector<double>{10.,100.,7.}),
        rclcpp::Parameter("sim_sensor_noise_std",std::vector<double>(7,0.))}));
    h.configure_estimator_noise();h.reset_vehicle_estimators();
    const std::string dir=TEST_CASADI_DIR;
    h.estimator_predict_=casadi::external("disturbance_ekf_predict",dir+"/libdisturbanceEKF_next.so");
    h.estimator_covariance_predict_=casadi::external("disturbance_ekf_predict_covariance",dir+"/libdisturbanceEKF_next.so");
    h.estimator_correct_=casadi::external("disturbance_ekf_correct",dir+"/libdisturbanceEKF_next.so");
    h.sensor_correct_=casadi::external("disturbance_ekf_sensor_correct",dir+"/libdisturbanceEKF_next.so");
    h.physical_sensor_model_=casadi::external("vehicle_sensor_model",dir+"/libdisturbanceEKF_next.so");
    h.utils_service.uv_Exkalman_update=casadi::external("disturbance_ekf_update",dir+"/libdisturbanceEKF_next.so");
    std::vector<double> truth(12,0.);truth[2]=2.;
    std::array<int,7> count{};
    std::array<double,7> previous{};
    for(int n=0;n<2000;++n) {
      truth[2]+=.001;truth[6]+=.0001;
      const auto y=h.sample_vehicle_measurements(truth,.005);
      for(int i=0;i<7;++i) {
        if(h.sensor_new_[i]) {++count[i]; EXPECT_DOUBLE_EQ(y[i],truth[i+2]);}
        else EXPECT_DOUBLE_EQ(y[i],previous[i]);
      }
      previous=y;
    }
    // Initial sample plus samples on each 0.1/0.01/1/7 second deadline.
    EXPECT_EQ(count[0],101);EXPECT_EQ(count[1],1001);EXPECT_EQ(count[4],71);
    for(int i=1;i<4;++i)EXPECT_EQ(count[i],count[1]);
    for(int i=4;i<7;++i)EXPECT_EQ(count[i],count[4]);
    // Exercise production read(): absent measurements must give prediction only,
    // including covariance, and a repeated read must do nothing.
    h.sim_step_=1.;h.hw_vehicle_struct.sim_period=.005;
    h.sensor_new_.fill(0.);
    h.x_est_(2)=2.;h.x_est_(6)=.2;
    const auto prior=h.estimator_predict_(std::vector<DM>{h.x_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.005});
    const auto continuous_prior=casadi_reach_alpha_5::predict_disturbance_covariance(
        h.estimator_covariance_predict_,h.x_est_,h.P_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.005,h.Q_);
    const DM expected_P=continuous_prior[1];
    ASSERT_EQ(h.read(rclcpp::Time(1),rclcpp::Duration::from_seconds(.005)),hardware_interface::return_type::OK);
    EXPECT_NEAR(double(DM::norm_inf(h.x_est_-prior[0])),0.,1e-12);
    EXPECT_NEAR(double(DM::norm_inf(h.P_est_-expected_P)),0.,1e-12);
    const auto corrected=h.x_est_;
    h.read(rclcpp::Time(2),rclcpp::Duration::from_seconds(.005));
    EXPECT_DOUBLE_EQ(double(DM::norm_inf(h.x_est_-corrected)),0.);
    // With all measurements fresh, the split path matches the existing export.
    h.sensor_new_.fill(1.);h.sim_step_=2.;
    h.hw_vehicle_struct.depth_from_pressure2=2.001;
    DM y=DM::zeros(7,1);y(0)=2.001;
    const auto next_prior=casadi_reach_alpha_5::predict_disturbance_covariance(
        h.estimator_covariance_predict_,h.x_est_,h.P_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.005,h.Q_);
    const auto expected=h.utils_service.uv_Exkalman_update(std::vector<DM>{
      h.x_est_,h.P_est_,h.estimator_applied_input_,h.estimator_nominal_parameters_,.005,y,next_prior[3],h.R_});
    h.read(rclcpp::Time(3),rclcpp::Duration::from_seconds(.005));
    EXPECT_NEAR(double(DM::norm_inf(h.x_est_-expected[0])),0.,1e-9);
    EXPECT_NEAR(double(DM::norm_inf(h.P_est_-expected[1])),0.,1e-9);
    const auto before=h.sensor_sample_time_;
    h.sample_vehicle_measurements(truth,.7);
    EXPECT_GT(h.sensor_sample_time_[2],before[2]); // one sample, no fabricated catch-up
    EXPECT_THROW(h.sample_vehicle_measurements(truth,0.),std::invalid_argument);
    h.sensor_elapsed_=0.;h.sensor_next_sample_.fill(0.);
    h.sensor_rates_={10.,200.,10.};
    int imu_samples=0;
    for(int i=0;i<2000;++i) {
      const double jitter=(i%4==0 || i%4==3) ? -1e-6 : 1e-6;
      h.sample_vehicle_measurements(truth,.005+jitter);
      imu_samples+=int(h.sensor_new_[1]);
    }
    EXPECT_EQ(imu_samples,2000); // same-rate sampling must not alias timing jitter

  }
  static void sensor_noise() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware h, repeat;
    auto options=rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("sim_sensor_rates_hz",std::vector<double>{0.,0.,0.}),
        rclcpp::Parameter("sim_sensor_noise_enabled",true),
        rclcpp::Parameter("sim_sensor_noise_seed",42),
        rclcpp::Parameter("sim_sensor_noise_std",std::vector<double>{.01,.002,.003,.004,.005,.006,.007})});
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("sensor_noise_test",options);
    repeat.node_topics_interface_=std::make_shared<rclcpp::Node>("sensor_noise_repeat_test",options);
    h.configure_estimator_noise();repeat.configure_estimator_noise();
    const std::vector<double> truth{0.,0.,2.,.1,-.2,3.2,.3,-.4,.5,0.,0.,0.};
    std::array<double,7> sum{},square{};
    for(int n=0;n<50000;++n) {
      const auto y=h.sample_vehicle_measurements(truth);
      const auto same=repeat.sample_vehicle_measurements(truth);
      for(int i=0;i<7;++i) {
        EXPECT_DOUBLE_EQ(y[i],same[i]);
        const double e=y[i]-truth[i+2];sum[i]+=e;square[i]+=e*e;
      }
    }
    for(int i=0;i<7;++i) {
      EXPECT_NEAR(sum[i]/50000.,0.,h.sensor_noise_std_[i]*.025);
      EXPECT_NEAR(square[i]/50000.,h.sensor_noise_std_[i]*h.sensor_noise_std_[i],h.sensor_noise_std_[i]*h.sensor_noise_std_[i]*.025);
      EXPECT_DOUBLE_EQ(double(h.R_(i,i)),1e-6);
    }
    // Switching injection preserves tuning and the seeded sampling sequence.
    const DM original_R=h.R_, original_sensor_R=h.sensor_R_;
    const auto rates=h.sensor_rates_;
    h.sensor_noise_requested_.store(false);
    EXPECT_DOUBLE_EQ(h.sensor_noise_enabled_,1.);
    for(int n=0;n<20;++n) {
      const auto ideal=h.sample_vehicle_measurements(truth);
      repeat.sample_vehicle_measurements(truth);
      for(int i=0;i<7;++i) EXPECT_DOUBLE_EQ(ideal[i],truth[i+2]);
    }
    EXPECT_DOUBLE_EQ(h.sensor_noise_enabled_,0.);
    EXPECT_DOUBLE_EQ(h.apply_sensor_noise(.123),0.);
    EXPECT_EQ(h.sensor_rates_,rates);
    EXPECT_DOUBLE_EQ(double(DM::norm_inf(h.R_-original_R)),0.);
    EXPECT_DOUBLE_EQ(double(DM::norm_inf(h.sensor_R_-original_sensor_R)),0.);
    h.sensor_noise_requested_.store(true);
    EXPECT_EQ(h.sample_vehicle_measurements(truth),repeat.sample_vehicle_measurements(truth));
    EXPECT_DOUBLE_EQ(h.apply_sensor_noise(.123),.123);
    // No sensor sample modifies the physical-state input.
    EXPECT_DOUBLE_EQ(truth[2],2.); EXPECT_DOUBLE_EQ(truth[5],3.2);
    for (const auto &bad:std::vector<std::vector<double>>{
        std::vector<double>(6,.1),std::vector<double>(7,-.1),
        std::vector<double>(7,std::numeric_limits<double>::quiet_NaN())}) {
      SimVehicleSystemMultiInterfaceHardware invalid;
      invalid.node_topics_interface_=std::make_shared<rclcpp::Node>("invalid_sensor_test",
          rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("sim_sensor_noise_std",bad)}));
      EXPECT_THROW(invalid.configure_estimator_noise(),std::invalid_argument);
    }
  }
  static void ideal_sensor_wrench_tracking() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware h;
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("ideal_sensor_wrench_test",
        rclcpp::NodeOptions().parameter_overrides({
          rclcpp::Parameter("sim_sensor_noise_std",std::vector<double>(7,0.0)),
          // This ideal-sensor test deliberately assumes ideal measurements too.
          rclcpp::Parameter("depth_attitude_measurement_variances",std::vector<double>(4,1e-18)),
          rclcpp::Parameter("dvl_measurement_covariance",std::vector<double>{1e-18,0.,0.,0.,1e-18,0.,0.,0.,1e-18})}));
    h.configure_estimator_noise();
    h.reset_vehicle_estimators();
    const std::string dir=TEST_CASADI_DIR;
    auto plant=casadi::external("Vnext_reg",dir+"/libUV_xnext.so");
    auto update=casadi::external("disturbance_ekf_update",dir+"/libdisturbanceEKF_next.so");
    DM state=DM::zeros(12,1); state(2)=2.;
    DM estimated=DM::vertcat({state,DM::zeros(6,1)}), covariance=h.P_est_;
    DM input=DM::zeros(6,1);
    double squared_error=0.; int count=0;
    for (int i=0;i<2000;++i) {
      const double dt=.005, t=i*dt;
      const double force=std::sin(2.*t), torque=.1*std::sin(.7*t);
      DM load=DM::zeros(6,1); load(0)=force; load(4)=torque-.2*force;
      state=plant(std::vector<DM>{state,input,h.estimator_nominal_parameters_,dt,load})[0];
      const auto result=update(std::vector<DM>{estimated,covariance,input,
          h.estimator_nominal_parameters_,dt,state(casadi::Slice(2,9)),h.Q_*dt,h.R_});
      estimated=result[0];covariance=result[1];
      for (const auto &value:result)
        for (double v:value.nonzeros()) ASSERT_TRUE(std::isfinite(v));
      if(i>400) {
        const double error=double(estimated(16))+.2*double(estimated(12))-torque;
        squared_error+=error*error; ++count;
      }
    }
    EXPECT_LT(std::sqrt(squared_error/count),.005); // N m at the offset load origin.
  }
  static void run() {
    using casadi::DM;
    SimVehicleSystemMultiInterfaceHardware h;
    h.node_topics_interface_=std::make_shared<rclcpp::Node>("retained_ekf_test");
    h.hw_vehicle_struct.robot_prefix="robot_1_";
    h.hw_vehicle_struct.sim_time=10.;
    h.hw_vehicle_struct.depth_from_pressure2=1.;
    h.sim_generation_=3.;
    h.reset_vehicle_estimators();
    h.x_est_(2)=1.; h.x_est_(6)=.5;
    const DM nominal=h.estimator_nominal_parameters_;
    h.vehicle_parameters_new={DM(123.)};
    SimVehicleSystemMultiInterfaceHardware::Identification m;
    m.stamp=h.node_topics_interface_->now();m.joint="robot_1_IOs";
    m.source_generation=3;m.epoch=7;
    m.retained_available=m.retained_valid=true;m.retained_accepted_time=9.;
    for(int i=0;i<27;++i)m.retained_parameters[i]=double(nominal(i));
    m.retained_parameters[6]=m.retained_parameters[3];
    m.retained_parameters[5]=m.retained_parameters[4];
    m.retained_parameters[0]*=1.5;
    // Candidate (non-retained) parameters deliberately unusable.
    m.parameters.fill(std::numeric_limits<double>::quiet_NaN());
    h.x_est_(12)=8.;h.P_est_(0,12)=h.P_est_(12,0)=.1;
    const DM navigation=h.x_est_(casadi::Slice(0,12));
    h.receive_retained_model(m);h.apply_retained_model();
    ASSERT_EQ(h.estimator_model_retained_,1.);
    EXPECT_EQ(h.estimator_model_epoch_,7.);
    EXPECT_DOUBLE_EQ(double(h.estimator_nominal_parameters_(0)),m.retained_parameters[0]);
    EXPECT_DOUBLE_EQ(double(h.vehicle_parameters_new[0]),123.);
    EXPECT_EQ(double(DM::norm_inf(h.x_est_(casadi::Slice(0,12))-navigation)),0.);
    EXPECT_EQ(double(h.x_est_(12)),0.);EXPECT_EQ(double(h.P_est_(0,12)),0.);
    EXPECT_EQ(double(h.P_est_(12,12)),1.);
    auto predict=casadi::external("disturbance_ekf_predict",std::string(TEST_CASADI_DIR)+"/libdisturbanceEKF_next.so");
    const DM input=DM::ones(6,1)*10.;
    const auto old_prediction=predict(std::vector<DM>{h.x_est_,input,nominal,.01});
    const auto new_prediction=predict(std::vector<DM>{h.x_est_,input,h.estimator_nominal_parameters_,.01});
    EXPECT_GT(double(DM::norm_inf(old_prediction[0]-new_prediction[0])),1e-6);
    for(double v:new_prediction[0].nonzeros())EXPECT_TRUE(std::isfinite(v));
    h.x_est_(12)=2.;
    h.receive_retained_model(m);h.apply_retained_model();
    EXPECT_EQ(double(h.x_est_(12)),2.); // Duplicate does not reset disturbance.
    auto reject=[&](auto bad){
      h.receive_retained_model(bad);EXPECT_FALSE(h.pending_retained_model_.has_value());
      h.apply_retained_model();EXPECT_EQ(double(h.x_est_(12)),2.);
    };
    auto bad=m;bad.retained_valid=false;reject(bad);
    bad=m;bad.hardware_measurements=true;reject(bad);
    bad=m;bad.source_generation=2;reject(bad);
    bad=m;bad.retained_accepted_time=8.;reject(bad);
    bad=m;bad.joint="robot_2_IOs";reject(bad);
    bad=m;bad.retained_parameters[0]=-1.;reject(bad);
    bad=m;bad.retained_parameters[0]=std::numeric_limits<double>::quiet_NaN();reject(bad);
    bad=m;bad.stamp=rclcpp::Time(0,0,RCL_ROS_TIME);reject(bad);
    h.receive_retained_model(m);
    h.retained_model_received_-=std::chrono::seconds(4);
    h.apply_retained_model();EXPECT_EQ(double(h.x_est_(12)),2.);
    m.retained_accepted_time=9.5;m.retained_net_weight_only=true;
    h.receive_retained_model(m);h.apply_retained_model();
    EXPECT_EQ(h.estimator_model_retained_,1.);
    h.hw_vehicle_struct.depth_from_pressure2=-.1;
    h.apply_retained_model();EXPECT_EQ(h.estimator_model_retained_,0.);
    EXPECT_EQ(double(DM::norm_inf(h.estimator_nominal_parameters_-nominal)),0.);
    h.hw_vehicle_struct.depth_from_pressure2=1.;
    h.receive_retained_model(m);h.apply_retained_model();
    ASSERT_EQ(h.estimator_model_retained_,1.);
    h.reset_vehicle_estimators();EXPECT_EQ(h.estimator_model_retained_,0.);
    EXPECT_EQ(double(DM::norm_inf(h.estimator_nominal_parameters_-nominal)),0.);
    h.receive_retained_model(m);EXPECT_FALSE(h.pending_retained_model_.has_value());
  }
};
}
TEST(VehiclePartialFeedback, RetainedSimulationEkfModel) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::run();
  rclcpp::shutdown();
}

TEST(VehiclePartialFeedback, IdealSimulationSensorWrenchTracking) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::ideal_sensor_wrench_tracking();
  rclcpp::shutdown();
}

TEST(VehiclePartialFeedback, SimulationSensorNoiseCovarianceAndSeed) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::sensor_noise();
  rclcpp::shutdown();
}

TEST(VehiclePartialFeedback, HardwareSensorCovarianceAndPressureReference) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::HardwareEstimatorTestAccess::covariance_and_pressure();
  rclcpp::shutdown();
}
TEST(VehiclePartialFeedback, MultiRateMeasurementConsumption) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::multirate();
  rclcpp::shutdown();
}

TEST(VehiclePartialFeedback, FullSimulatedDvlCovariance) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::correlated_dvl();
  rclcpp::shutdown();
}

TEST(VehiclePartialFeedback, FullWrenchProcessCovariance) {
    const std::vector<double> diagonal(6,0.01);
    auto expected=casadi::DM::diag(casadi::DM(diagonal));
    expected(0,4)=expected(4,0)=0.003;
    std::vector<double> full(36);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)full[6*i+j]=double(expected(i,j));
    const auto actual=casadi_reach_alpha_5::wrench_process_covariance(full,diagonal);
    EXPECT_DOUBLE_EQ(double(casadi::DM::norm_inf(actual-expected)),0.);
    EXPECT_DOUBLE_EQ(double(casadi_reach_alpha_5::wrench_process_covariance({},diagonal)(0,0)),.01);
    EXPECT_THROW(casadi_reach_alpha_5::wrench_process_covariance({1.},diagonal),std::invalid_argument);
    full[0]=-1.;
    EXPECT_THROW(casadi_reach_alpha_5::wrench_process_covariance(full,diagonal),std::invalid_argument);
    full[0]=.01;full[4]=.004;
    EXPECT_THROW(casadi_reach_alpha_5::wrench_process_covariance(full,diagonal),std::invalid_argument);
}

TEST(VehiclePartialFeedback, ContinuousNoisePredictionSubsteps) {
    using casadi::DM;
    const auto function=casadi::external("disturbance_ekf_predict_covariance",
        std::string(TEST_CASADI_DIR)+"/libdisturbanceEKF_next.so");
    const auto nominal=casadi::external("disturbance_ekf_predict",
        std::string(TEST_CASADI_DIR)+"/libdisturbanceEKF_next.so");
    DM x=DM::zeros(18,1);x(2)=2.;x(6)=.1;
    const DM P=DM::eye(18)*.001, u=DM::zeros(6,1);
    const DM parameters(ros2_control_blue_reach_5::default_nominal_vehicle_parameters());
    DM Q=DM::zeros(18,18);
    for(int i=12;i<18;++i)Q(i,i)=.0007;
    const auto result=casadi_reach_alpha_5::predict_disturbance_covariance(
        function,x,P,u,parameters,.05,Q);
    const auto expected=nominal(std::vector<DM>{x,u,parameters,.05});
    EXPECT_NEAR(double(DM::norm_inf(result[0]-expected[0])),0.,1e-10);
    const DM reconstructed=DM::mtimes(std::vector<DM>{result[2],P,result[2].T()})+result[3];
    EXPECT_NEAR(double(DM::norm_inf(result[1]-reconstructed)),0.,1e-12);
    EXPECT_GT(double(DM::norm_inf(result[3](casadi::Slice(6,12),casadi::Slice(12,18)))),0.);
    EXPECT_THROW(casadi_reach_alpha_5::predict_disturbance_covariance(
        function,x,P,u,parameters,0.,Q),std::invalid_argument);
    EXPECT_THROW(casadi_reach_alpha_5::predict_disturbance_covariance(
        function,x,P,u,parameters,std::numeric_limits<double>::quiet_NaN(),Q),std::invalid_argument);
}

TEST(VehiclePartialFeedback, AdaptationRejectsFullCovarianceOverride) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::HardwareEstimatorTestAccess::adaptation_configuration();
  rclcpp::shutdown();
}


TEST(VehiclePartialFeedback, IndependentMeasurementNoiseAssumptions) {
  rclcpp::init(0,nullptr);
  ros2_control_blue_reach_5::SimEstimatorTestAccess::independent_measurement_noise();
  rclcpp::shutdown();
}
