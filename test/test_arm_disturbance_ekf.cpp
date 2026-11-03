// Native estimator tests: no simulator process or actuator commands.
#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/sim_reach_system_multi_interface.hpp"

namespace ros2_control_blue_reach_5 {
struct ArmEstimatorTestAccess {
  static void run() {
    using casadi::DM;
    using casadi::Slice;
    SimReachSystemMultiInterfaceHardware h;
    h.is_locked_.assign(5, false);
    h.on_db_.assign(5, .05);
    h.off_db_.assign(5, .10);
    h.arm_torques.assign(5, DM(0.));
    h.update_joint_locks();
    for (int j=0;j<4;++j) EXPECT_TRUE(h.is_locked_[j]);
    h.arm_torques[1]=-.11;
    h.update_joint_locks();
    EXPECT_FALSE(h.is_locked_[1]);
    h.arm_torques[1]=-.07;
    h.update_joint_locks();
    EXPECT_FALSE(h.is_locked_[1]); // original hysteresis is unchanged
    h.arm_joint_locks_enabled_=false;
    h.arm_torques.assign(5, DM(0.));
    h.update_joint_locks();
    for (bool locked:h.is_locked_) EXPECT_FALSE(locked);
    h.is_locked_.assign(5,true);
    h.on_db_.assign(5,100.); // profile edits cannot re-enable startup-disabled locks
    h.update_joint_locks();
    for (bool locked:h.is_locked_) EXPECT_FALSE(locked);
    // Candidate current/torque is freshly computed each write, before gating.
    h.hw_joint_struct_.resize(5);
    h.arm_joint_locks_enabled_=true;
    h.on_db_.assign(5,.05);
    h.off_db_.assign(5,.10);
    h.is_locked_.assign(5,false);
    auto request = [&](double torque) {
      h.arm_torques.assign(5,DM(torque));
      for(auto &joint:h.hw_joint_struct_) {
        joint.current_state_.current=.4;
        joint.current_state_.effort=torque;
        joint.current_state_.computed_effort=torque;
      }
      h.apply_motor_lock();
    };
    request(.04);
    for(int j=0;j<4;++j) {
      EXPECT_TRUE(h.is_locked_[j]);
      EXPECT_EQ(double(h.arm_torques[j]),0.);
      EXPECT_EQ(h.hw_joint_struct_[j].current_state_.current,0.);
      EXPECT_EQ(h.hw_joint_struct_[j].current_state_.effort,0.);
      EXPECT_EQ(h.hw_joint_struct_[j].current_state_.computed_effort,.04);
    }
    EXPECT_EQ(double(h.arm_torques[4]),.04); // gripper not gated
    request(-.07); // locked hysteresis band remains motor-off
    EXPECT_EQ(double(h.arm_torques[0]),0.);
    request(-.11); // unlock from a fresh limited request, not zero feedback
    EXPECT_FALSE(h.is_locked_[0]);
    EXPECT_EQ(double(h.arm_torques[0]),-.11);
    EXPECT_EQ(h.hw_joint_struct_[0].current_state_.current,.4);
    request(-.07);
    EXPECT_FALSE(h.is_locked_[0]);
    EXPECT_EQ(double(h.arm_torques[0]),-.07);
    request(0.); // includes requests removed by the current/position limiter
    EXPECT_TRUE(h.is_locked_[0]);
    h.arm_joint_locks_enabled_=false;
    request(.04);
    EXPECT_FALSE(h.is_locked_[0]);
    EXPECT_EQ(double(h.arm_torques[0]),.04);
    const DM actuator(std::vector<double>{1., 2., 3., 4., 5.});
    EXPECT_EQ(double(DM::norm_inf(h.plant_torque_with_disturbance(actuator)-actuator)), 0.);
    std_msgs::msg::Float64MultiArray injected;
    injected.data = {.1, -.2, .3, -.4};
    h.receive_external_torque(injected);
    auto total = h.plant_torque_with_disturbance(actuator);
    EXPECT_NEAR(double(total(0)), 1.1, 1e-12);
    EXPECT_NEAR(double(total(3)), 3.6, 1e-12);
    EXPECT_EQ(double(total(4)), 5.);  // gripper is not a disturbance DOF
    EXPECT_EQ(double(actuator(0)), 1.);  // actuator input is never modified
    injected.data = {99.};
    h.receive_external_torque(injected);
    EXPECT_NEAR(double(h.plant_torque_with_disturbance(actuator)(0)), 1.1, 1e-12);
    h.external_torque_received_ -= std::chrono::seconds(2);
    EXPECT_EQ(double(DM::norm_inf(h.plant_torque_with_disturbance(actuator)-actuator)), 0.);
    h.info_.joints.resize(5);
    h.hw_joint_struct_.resize(5);
    const std::string dir = TEST_CASADI_DIR;
    auto predict = casadi::external("arm_disturbance_ekf_predict",
                                    dir + "/libmDisturbanceEKF_next.so");
    auto plant = casadi::external("Mnext_reg", dir + "/libMnext.so");
    h.utils_service.manip_disturbance_predict = casadi::external(
        "arm_disturbance_ekf_predict_locked", dir + "/libmDisturbanceEKF_next.so");
    h.utils_service.manip_disturbance_update = casadi::external(
        "arm_disturbance_ekf_update_limited", dir + "/libmDisturbanceEKF_next.so");
    h.utils_service.manip_Exkalman_update = casadi::external("ekf_update", dir + "/libmEKF_next.so");
    h.utils_service.forward_kinematics = casadi::external("fkeval", dir + "/libFK.so");
    h.utils_service.forward_kinematics_com = casadi::external("fkcomeval", dir + "/libFKcom.so");
    for (auto &joint:h.hw_joint_struct_) {
      joint.limits_.position_min=-100.;
      joint.limits_.position_max=100.;
    }
    h.reset_joint_estimators();
    for (int i = 0; i < 5; ++i) {
      h.hw_joint_struct_[i].current_state_.position = i == 0 ? 3.0 : 1.0;
      h.seed_joint_estimator_from_state(i, h.hw_joint_struct_[i].current_state_);
    }
    const auto nominal = h.estimator_nominal_parameters_;
    const double dt = .01;
    const auto now = rclcpp::Time(1);
    // Caller read period deliberately differs from completed integration dt.
    const auto read_period = rclcpp::Duration::from_seconds(.04);
    EXPECT_EQ(h.read(now, read_period), hardware_interface::return_type::OK);
    EXPECT_EQ(h.estimator_valid_, 0.);
    const DM input = DM(std::vector<double>{.2, -.15, .1, -.08});
    const DM initial = h.arm_x_est_;
    const auto prediction = predict(std::vector<DM>{initial, input, nominal, dt});
    auto predict_locked = casadi::external("arm_disturbance_ekf_predict_locked",
                                           dir + "/libmDisturbanceEKF_next.so");
    // Four-stage predictor versus 30-stage plant: discretization bounds below
    // are well below measurement standard deviations (1e-3 rad, sqrt(1e-5) rad/s).
    DM plant_state = DM::zeros(10, 1), plant_input = DM::zeros(5, 1);
    plant_state(Slice(0, 4)) = initial(Slice(0, 4));
    plant_state(Slice(5, 9)) = initial(Slice(4, 8));
    plant_input(Slice(0, 4)) = input;
    const auto stepped = plant(std::vector<DM>{plant_state, plant_input, dt,
        nominal, 300., 400., 0., DM::zeros(4, 1), 200.})[0];
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(double(prediction[0](i)), double(stepped(i)), 1e-7);
      EXPECT_NEAR(double(prediction[0](4+i)), double(stepped(5+i)), 2e-6);
    }
    // Every lock combination must agree with the production constrained plant,
    // including a moving joint being braked by the Baumgarte constraint.
    for (int bits = 0; bits < 16; ++bits) {
      DM mask = DM::zeros(4), moving = initial;
      for (int j = 0; j < 4; ++j) {
        mask(j) = (bits >> j) & 1;
        moving(4+j) = .02 * (j+1);
      }
      DM ps = plant_state;
      ps(Slice(5,9)) = moving(Slice(4,8));
      auto actual = plant(std::vector<DM>{ps, plant_input, dt, nominal,
          300.,400.,0.,mask,200.})[0];
      auto predicted = predict_locked(std::vector<DM>{moving,input,nominal,dt,mask,200.});
      EXPECT_LT(double(DM::norm_inf(predicted[0](Slice(0,4))-actual(Slice(0,4)))), 1e-7);
      EXPECT_LT(double(DM::norm_inf(predicted[0](Slice(4,8))-actual(Slice(5,9)))), 2e-6);
      for (int j = 0; j < 12; ++j) {
        DM plus=moving, minus=moving;
        plus(j)+=1e-6; minus(j)-=1e-6;
        DM numerical=(predict_locked(std::vector<DM>{plus,input,nominal,dt,mask,200.})[0]-
                      predict_locked(std::vector<DM>{minus,input,nominal,dt,mask,200.})[0])/2e-6;
        EXPECT_LT(double(DM::norm_inf(numerical-predicted[1](Slice(),j))),1e-6);
      }
      DM prior = DM::eye(12), noise = DM::eye(12)*.001;
      auto post = h.utils_service.manip_disturbance_update(std::vector<DM>{
          moving,prior,input,nominal,dt,predicted[0](Slice(0,8)),noise,
          DM::eye(8)*1e-5,mask,200.,h.estimator_lower_limits_,h.estimator_upper_limits_,DM::zeros(4)});
      for (int j = 0; j < 4; ++j) if ((bits >> j) & 1) {
        EXPECT_EQ(double(post[0](8+j)), double(moving(8+j)));
        EXPECT_NEAR(double(post[1](8+j,8+j)), 1.001, 1e-10);
      }
    }
    auto limited = casadi::external("arm_disturbance_ekf_predict_limited",
                                    dir + "/libmDisturbanceEKF_next.so");
    // Compare against the production plant followed by its actual clamp rule,
    // for lower and upper impacts at each of the four joints.
    for (int axis=0;axis<4;++axis) for (int sign:{-1,1}) {
      DM moving=initial,lo=DM::ones(4)*-100.,hi=DM::ones(4)*100.;
      moving(4+axis)=sign*1.;
      if(sign<0)lo(axis)=double(initial(axis))-.001;
      else hi(axis)=double(initial(axis))+.001;
      DM ps=plant_state;
      ps(Slice(5,9))=moving(Slice(4,8));
      DM actual=plant(std::vector<DM>{ps,plant_input,dt,nominal,300.,400.,0.,DM::zeros(4),200.})[0];
      for(int j=0;j<4;++j) {
        if(double(actual(j))<double(lo(j))) {
          actual(j)=lo(j);actual(5+j)=std::max(0.,double(actual(5+j)));
        } else if(double(actual(j))>double(hi(j))) {
          actual(j)=hi(j);actual(5+j)=std::min(0.,double(actual(5+j)));
        }
      }
      auto prediction=limited(std::vector<DM>{moving,input,nominal,dt,DM::zeros(4),200.,lo,hi});
      EXPECT_LT(double(DM::norm_inf(prediction[0](Slice(0,4))-actual(Slice(0,4)))),1e-7);
      EXPECT_LT(double(DM::norm_inf(prediction[0](Slice(4,8))-actual(Slice(5,9)))),2e-6);
      for (int j=0;j<12;++j) {
        DM plus=moving,minus=moving;
        plus(j)+=1e-6;minus(j)-=1e-6;
        DM numerical=(limited(std::vector<DM>{plus,input,nominal,dt,DM::zeros(4),200.,lo,hi})[0]-
                      limited(std::vector<DM>{minus,input,nominal,dt,DM::zeros(4),200.,lo,hi})[0])/2e-6;
        EXPECT_LT(double(DM::norm_inf(numerical-prediction[1](Slice(),j))),1e-6);
      }
      DM contact=DM::zeros(4);contact(axis)=1.;
      auto post=h.utils_service.manip_disturbance_update(std::vector<DM>{
          moving,DM::eye(12),input,nominal,dt,prediction[0](Slice(0,8)),
          DM::eye(12)*.001,DM::eye(8)*1e-5,DM::zeros(4),200.,lo,hi,contact});
      EXPECT_EQ(double(DM::norm_inf(post[5])),0.); // contact closes full-vector controller admission
      for(int j=0;j<4;++j) {
        EXPECT_EQ(double(post[0](8+j)),double(moving(8+j)));
        EXPECT_LE(double(post[1](8+j,8+j)),1.001+1e-10);
        EXPECT_GE(double(post[1](8+j,8+j)),0.);
      }
    }
    // Censored contact coordinates are not torque evidence, even when the
    // nominal model predicts no contact. Free-coordinate evidence is retained.
    {
      DM state=initial,P=DM::eye(12),zero=DM::zeros(4),contact=DM::zeros(4);
      contact(2)=1.;
      for(int k=0;k<20;++k) {
        auto prediction=predict(std::vector<DM>{state,input,nominal,dt})[0];
        DM y=prediction(Slice(0,8));y(6)=0.;
        auto post=h.utils_service.manip_disturbance_update(std::vector<DM>{
          state,P,input,nominal,dt,y,DM::eye(12)*.001,DM::eye(8)*1e-5,
          zero,200.,h.estimator_lower_limits_,h.estimator_upper_limits_,contact});
        state=post[0];P=post[1];
        EXPECT_EQ(double(DM::norm_inf(state(Slice(8,12)))),0.);
        EXPECT_EQ(double(DM::norm_inf(post[5])),0.);
      }
      EXPECT_LT(double(DM::trace(P(Slice(8,12),Slice(8,12)))),4.08);
      auto post=h.utils_service.manip_disturbance_update(std::vector<DM>{
          state,P,input,nominal,dt,state(Slice(0,8)),DM::eye(12)*.001,DM::eye(8)*1e-5,
          zero,200.,h.estimator_lower_limits_,h.estimator_upper_limits_,zero});
      EXPECT_EQ(double(DM::norm_inf(post[5]-DM::ones(4))),0.);
    }
    // Regression: the prior predicts an impact but the measured plant has
    // released. A prediction-selected contact mode would permanently disable
    // the disturbance sensitivity needed to explain that release.
    {
      DM state=initial,lo=DM::ones(4)*-100.,hi=DM::ones(4)*100.;
      state(7)=-.2;lo(3)=double(state(3));
      DM free=predict_locked(std::vector<DM>{state,input,nominal,dt,DM::zeros(4),200.})[0];
      ASSERT_LT(double(free(3)),double(lo(3)));
      DM y=free(Slice(0,8));y(3)=double(lo(3))+.002;y(7)=.1;
      auto post=h.utils_service.manip_disturbance_update(std::vector<DM>{
          state,DM::eye(12)*80.,input,nominal,dt,y,
          DM::eye(12)*.001,DM::eye(8)*1e-5,DM::zeros(4),200.,lo,hi,DM::zeros(4)});
      EXPECT_EQ(double(DM::norm_inf(post[5]-DM::ones(4))),0.);
      EXPECT_GT(double(post[0](3)),double(lo(3)));
      EXPECT_GT(std::abs(double(post[0](11))),.001);
      EXPECT_LT(double(post[1](11,11)),80.);
    }
    // Check AD through the nonlinear step, including disturbance columns.
    for (int j = 0; j < 12; ++j) {
      DM plus = initial, minus = initial;
      plus(j) += 1e-6; minus(j) -= 1e-6;
      const DM numerical = (predict(std::vector<DM>{plus,input,nominal,dt})[0] -
                            predict(std::vector<DM>{minus,input,nominal,dt})[0]) / 2e-6;
      EXPECT_LT(double(DM::norm_inf(numerical-prediction[1](Slice(),j))), 1e-6);
    }
    // Applied effort plus a hidden additive load drives truth. Only measured
    // q/dq and applied effort reach read(); the load is not an observer input.
    // Independent original production plant is the observation oracle, not
    // the EKF predictor. Retain the hidden load only in the test's truth state.
    auto advance_truth = [&](DM state, const DM &applied, const DM &parameters, const DM &mask) {
      DM z=DM::zeros(10),u=DM::zeros(5);
      z(Slice(0,4))=state(Slice(0,4)); z(Slice(5,9))=state(Slice(4,8));
      u(Slice(0,4))=applied+state(Slice(8,12));
      auto next=plant(std::vector<DM>{z,u,dt,parameters,300.,400.,0.,mask,200.})[0];
      state(Slice(0,4))=next(Slice(0,4));state(Slice(4,8))=next(Slice(5,9));
      return state;
    };
    const DM load = DM(std::vector<double>{.12, -.09, .06, -.04});
    DM truth = initial;
    truth(Slice(8, 12)) = load;
    for (int step = 1; step <= 400; ++step) {
      const DM applied = input * std::sin(step * dt);
      truth = advance_truth(truth,applied,nominal,DM::zeros(4));
      for (int i = 0; i < 4; ++i) {
        h.hw_joint_struct_[i].current_state_.position = double(truth(i));
        h.hw_joint_struct_[i].current_state_.velocity = double(truth(4+i));
      }
      h.estimator_applied_input_ = applied;
      h.estimator_completed_dt_ = dt;
      h.sim_step_ = step;
      ASSERT_EQ(h.read(now, read_period), hardware_interface::return_type::OK);
      EXPECT_EQ(h.estimator_step_, step);
      EXPECT_EQ(h.estimator_valid_, 1.);
      for (double variance : h.disturbance_variance_) EXPECT_GE(variance, 0.);
    }
    EXPECT_LT(double(DM::norm_inf(h.arm_x_est_(Slice(8,12))-load)), .005);
    // Load changes while locked cannot be inferred from motion. Preserve the
    // previous posterior mean, grow uncertainty, then recover after release.
    const DM held = h.arm_x_est_(Slice(8,12));
    const DM before_lock = h.arm_P_est_;
    truth(Slice(8,12)) = -load;
    for (int step = 401; step <= 700; ++step) {
      const DM mask = step <= 450 ? DM::ones(4) : DM::zeros(4);
      truth = advance_truth(truth,input,nominal,mask);
      for (int i=0; i<4; ++i) {
        h.hw_joint_struct_[i].current_state_.position=double(truth(i));
        h.hw_joint_struct_[i].current_state_.velocity=double(truth(4+i));
      }
      h.estimator_completed_lock_mask_=mask;
      h.estimator_applied_input_=input;
      h.sim_step_=step;
      ASSERT_EQ(h.read(now,read_period),hardware_interface::return_type::OK);
      if (step<=450) {
        EXPECT_LT(double(DM::norm_inf(h.arm_x_est_(Slice(8,12))-held)),1e-12);
        for(int i=0;i<4;++i) {
          EXPECT_EQ(h.estimator_joint_locked_[i],1.);
          EXPECT_NEAR(double(h.arm_P_est_(8+i,8+i)),
              double(before_lock(8+i,8+i))+(step-400)*double(h.arm_Q_(8+i,8+i))*dt,1e-9);
        }
      }
    }
    EXPECT_LT(double(DM::norm_inf(h.arm_x_est_(Slice(8,12))+load)),.005);
    for(double locked:h.estimator_joint_locked_) EXPECT_EQ(locked,0.);
    h.estimator_completed_contact_(2)=1.;
    h.sim_step_+=1;
    ASSERT_EQ(h.read(now,read_period),hardware_interface::return_type::OK);
    EXPECT_EQ(h.estimator_joint_at_limit_[2],1.);
    for(double available:h.estimator_disturbance_observable_)EXPECT_EQ(available,0.);
    EXPECT_EQ(h.estimator_valid_,1.);
    h.estimator_completed_contact_=DM::zeros(4);
    h.sim_step_+=1;
    ASSERT_EQ(h.read(now,read_period),hardware_interface::return_type::OK);
    for(double available:h.estimator_disturbance_observable_)EXPECT_EQ(available,1.);
    const DM state = h.arm_x_est_, covariance = h.arm_P_est_;
    EXPECT_EQ(h.read(now, read_period), hardware_interface::return_type::OK);
    EXPECT_EQ(double(DM::norm_inf(h.arm_x_est_-state)), 0.);
    EXPECT_EQ(double(DM::norm_inf(h.arm_P_est_-covariance)), 0.);
    h.sim_step_ += 2;
    EXPECT_EQ(h.read(now, read_period), hardware_interface::return_type::ERROR);
    EXPECT_EQ(h.estimator_valid_, 0.);
    // A plant profile change cannot leak into the nominal observer model.
    h.manipulator_parameters_ = std::vector<DM>(81, DM(99.));
    EXPECT_EQ(double(DM::norm_inf(h.estimator_nominal_parameters_-nominal)), 0.);
    h.reset_joint_simulation_state();
    EXPECT_EQ(double(DM::norm_inf(h.plant_torque_with_disturbance(actuator)-actuator)), 0.);
    EXPECT_EQ(h.estimator_valid_, 0.);
    EXPECT_EQ(h.estimator_step_, 0.);
    EXPECT_EQ(h.sim_step_, 0.);
    EXPECT_EQ(h.sim_generation_, 1.);
    for (double value : h.estimated_disturbance_) EXPECT_EQ(value, 0.);
    EXPECT_EQ(double(DM::norm_inf(h.estimator_nominal_parameters_-nominal)), 0.);
    // Pure viscous-friction mismatch is an unknown additive load relative to
    // the fixed nominal model, even when no external torque is injected.
    DM changed_plant = nominal;
    changed_plant(Slice(40,44)) = 1.5 * nominal(Slice(40,44));
    truth = initial;
    for (int i = 0; i < 4; ++i) {
      h.hw_joint_struct_[i].current_state_.position = double(initial(i));
      h.seed_joint_estimator_from_state(i, h.hw_joint_struct_[i].current_state_);
    }
    for (int step = 1; step <= 400; ++step) {
      truth = advance_truth(truth,input,changed_plant,DM::zeros(4));
      for (int i = 0; i < 4; ++i) {
        h.hw_joint_struct_[i].current_state_.position = double(truth(i));
        h.hw_joint_struct_[i].current_state_.velocity = double(truth(4+i));
      }
      h.estimator_applied_input_ = input;
      h.estimator_completed_dt_ = dt;
      h.sim_step_ = step;
      ASSERT_EQ(h.read(now, read_period), hardware_interface::return_type::OK);
    }
    const DM expected_load = -.5 * nominal(Slice(40,44)) * truth(Slice(4,8));
    EXPECT_LT(double(DM::norm_inf(h.arm_x_est_(Slice(8,12))-expected_load)), .005);
    EXPECT_EQ(double(DM::norm_inf(h.estimator_nominal_parameters_-nominal)), 0.);
  }
};
}

TEST(ArmDisturbanceEKF, NativeModelTimingAndReset) {
  rclcpp::init(0, nullptr);
  ros2_control_blue_reach_5::ArmEstimatorTestAccess::run();
  rclcpp::shutdown();
}

#include "ros2_control_blue_reach_5/arm_time_integration.hpp"
#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"
TEST(ArmDisturbanceEKF, SchedulingGapUsesSamePlantAndPredictionIntervals) {
  using namespace ros2_control_blue_reach_5;
  using casadi::DM; using casadi::Slice;
  auto plant=casadi::external("arm_plant_next",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
  auto predict=casadi::external("arm_disturbance_ekf_predict_locked",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
  auto correct=casadi::external("arm_disturbance_ekf_update_limited",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
  auto model=DM::vertcat(pack_manipulator_dynamics(default_manipulator_dynamics()));
  for (double duration : {.005,.05,.083,1.11499651,2.}) {
    for(int bits : {0,5,15}) {
      DM mask=DM::zeros(4),x=DM::zeros(12),z=DM::zeros(10);
      for(int j=0;j<4;++j) {mask(j)=(bits>>j)&1; x(j)=j==0?3.1:1.; x(4+j)=.01; z(j)=x(j);z(5+j)=x(4+j);}
      auto truth=arm_time_integration::plant(plant,
        {z,DM::zeros(5),duration,model,300.,400.,0.,mask,200.})[0];
      auto y=DM::vertcat({truth(Slice(0,4)),truth(Slice(5,9))});
      auto out=arm_time_integration::posterior(predict,correct,
        {x,DM::eye(12)*.001,DM::zeros(4),model,duration,y,DM::eye(12)*1e-6*duration,
         DM::eye(8)*1e-5,mask,200.,DM::ones(4)*-100.,DM::ones(4)*100.,DM::zeros(4)});
      for(auto &v:out) for(double d:v.nonzeros()) ASSERT_TRUE(std::isfinite(d));
      EXPECT_LT(double(DM::norm_inf(out[0](Slice(0,8))-y)),1e-6);
      EXPECT_LT(double(DM::norm_inf(out[1]-out[1].T())),1e-12);
      for(int i=0;i<12;++i) EXPECT_GE(double(out[1](i,i)),0.);
      if(bits==15) { EXPECT_LT(double(DM::norm_inf(out[0](Slice(8,12)))),1e-12); }
    }
  }
  EXPECT_THROW(arm_time_integration::intervals(0.,200.),std::runtime_error);
  EXPECT_THROW(arm_time_integration::intervals(6.,200.),std::runtime_error);
  EXPECT_THROW(arm_time_integration::intervals(NAN,200.),std::runtime_error);
  EXPECT_THROW(arm_time_integration::intervals(.01,-1.),std::runtime_error);
}

TEST(ArmDisturbanceEKF, ReducedPlantPreservesOriginalRK4AndGripper) {
  using namespace ros2_control_blue_reach_5;
  using casadi::DM;
  auto original=casadi::external("Mnext_reg",TEST_CASADI_DIR "/libMnext.so");
  auto reduced=casadi::external("arm_plant_next",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
  auto model=DM::vertcat(pack_manipulator_dynamics(default_manipulator_dynamics()));
  for(int bits=0;bits<16;++bits) for(double dt:{.005,.01}) {
    DM mask=DM::zeros(4),z=DM::zeros(10);
    for(int j=0;j<4;++j) {mask(j)=(bits>>j)&1;z(j)=.5+.3*j;z(j+5)=.01*(j+1);}
    z(4)=.006;z(9)=.001;
    std::vector<DM> args={z,DM(std::vector<double>{.1,-.05,.07,-.02,.5}),dt,model,300.,400.,2.,mask,200.};
    EXPECT_LT(double(DM::norm_inf(original(args)[0]-reduced(args)[0])),1e-10);
  }
}
