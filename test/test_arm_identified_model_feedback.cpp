#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"
#include "ros2_control_blue_reach_5/sim_reach_system_multi_interface.hpp"
#include <gtest/gtest.h>

namespace ros2_control_blue_reach_5 {
struct ArmEstimatorTestAccess {
  using Hardware = SimReachSystemMultiInterfaceHardware;
  using Message = Hardware::ArmIdentification;
  static void initialize(Hardware &h) {
    h.node_frames_interface_ =
        std::make_shared<rclcpp::Node>("arm_model_handover_test");
    h.info_.joints.resize(5);
    h.info_.gpios.resize(1);
    h.info_.gpios[0].name = "robot_1__arm_IOs";
    h.hw_joint_struct_.resize(5);
    for (auto &j : h.hw_joint_struct_) {
      j.limits_.position_min = -100;
      j.limits_.position_max = 100;
    }
    h.reset_joint_estimators();
    h.sim_generation_ = 4;
    h.manipulator_parameters_ =
        pack_manipulator_dynamics(default_manipulator_dynamics());
    h.identification_sample_.supported = 1;
    h.identification_sample_.time = 100;
    for (int j = 0; j < 8; ++j)
      h.arm_x_est_(j) = .03 * (j + 1);
    h.arm_P_est_ = casadi::DM::eye(12) * .4;
    h.arm_P_est_(0, 4) = h.arm_P_est_(4, 0) = .02;
  }
  static Message message(Hardware &h, bool gripper = false) {
    Message m;
    m.stamp = h.node_frames_interface_->now();
    m.joint = "robot_1__arm_IOs";
    m.subsystem = gripper ? "gripper" : "arm";
    m.source_generation = 4;
    m.epoch = 100;
    m.retained_valid = m.measurements_fresh = true;
    m.retained_accepted_time = 80;
    m.observation_time = 100;
    m.model_context = h.identification_sample_.context;
    if (gripper) {
      m.retained_parameters = {320, 380, 20};
      return m;
    }
    auto p = default_manipulator_dynamics();
    m.retained_parameters.resize(48);
    for (int j = 0; j < 4; ++j) {
      m.retained_parameters[12 * j] = p.link_masses[j] * 1.2;
      for (int k = 0; k < 3; ++k)
        m.retained_parameters[12 * j + 1 + k] =
            p.link_first_moments[3 * j + k] * 1.2;
      for (int k = 0; k < 6; ++k)
        m.retained_parameters[12 * j + 4 + k] =
            p.link_inertias[6 * j + k] * 1.2;
      m.retained_parameters[12 * j + 10] = p.viscous_friction[j] * .8;
      m.retained_parameters[12 * j + 11] = 0;
    }
    return m;
  }
  static void handover() {
    Hardware h;
    initialize(h);
    auto m = message(h);
    const auto state = h.arm_x_est_;
    const auto covariance = h.arm_P_est_;
    const auto old = h.estimator_nominal_parameters_;
    const auto plant = h.manipulator_parameters_;
    h.receive_identified_model(m);
    EXPECT_EQ(h.identification_ack_[0], 0);
    EXPECT_EQ(
        double(casadi::DM::norm_inf(h.estimator_nominal_parameters_ - old)), 0);
    h.apply_identified_models();
    ASSERT_EQ(h.identification_ack_[0], 1);
    EXPECT_EQ(h.identification_ack_[1], 100);
    EXPECT_EQ(h.identification_ack_[2], 80);
    for (int j = 0; j < 8; ++j) {
      EXPECT_DOUBLE_EQ(double(h.arm_x_est_(j)), double(state(j)));
      for (int k = 0; k < 8; ++k)
        EXPECT_DOUBLE_EQ(double(h.arm_P_est_(j, k)), double(covariance(j, k)));
    }
    for (int j = 0; j < 4; ++j) {
      EXPECT_DOUBLE_EQ(h.estimator_model_readback_[j],
                       m.retained_parameters[12 * j]);
      for (int k = 0; k < 3; ++k)
        EXPECT_DOUBLE_EQ(h.estimator_model_readback_[4 + 3 * j + k],
                         m.retained_parameters[12 * j + 1 + k]);
      for (int k = 0; k < 6; ++k)
        EXPECT_DOUBLE_EQ(h.estimator_model_readback_[16 + 6 * j + k],
                         m.retained_parameters[12 * j + 4 + k]);
      EXPECT_DOUBLE_EQ(h.estimator_model_readback_[40 + j],
                       m.retained_parameters[12 * j + 10]);
    }
    for (int i = 0; i < 81; ++i)
      EXPECT_DOUBLE_EQ(double(h.manipulator_parameters_[i]), double(plant[i]));
    h.arm_x_est_(8) = .7;
    m.stamp = h.node_frames_interface_->now();
    h.receive_identified_model(m);
    h.apply_identified_models();
    EXPECT_DOUBLE_EQ(double(h.arm_x_est_(8)),
                     .7); // Repeated acknowledgement is not a new handover.
    // A paused simulator may keep its already-applied model with a live
    // heartbeat.
    m.measurements_fresh = false;
    h.identified_model_received_[0] -= std::chrono::seconds(4);
    m.stamp = h.node_frames_interface_->now();
    h.receive_identified_model(m);
    h.apply_identified_models();
    EXPECT_EQ(h.identification_ack_[0], 1);
    EXPECT_DOUBLE_EQ(double(h.arm_x_est_(8)), .7);
    m.retained_valid = false;
    m.stamp = h.node_frames_interface_->now();
    h.receive_identified_model(m);
    h.apply_identified_models();
    EXPECT_EQ(h.identification_ack_[0], 0);
    EXPECT_EQ(
        double(casadi::DM::norm_inf(h.estimator_nominal_parameters_ - old)), 0);
    for (int j = 0; j < 8; ++j)
      EXPECT_DOUBLE_EQ(double(h.arm_x_est_(j)), double(state(j)));
  }
  static void reject() {
    Hardware h;
    initialize(h);
    for (int test = 0; test < 8; ++test) {
      auto m = message(h);
      if (test == 0)
        m.source_generation = 3;
      if (test == 1)
        m.stamp = h.node_frames_interface_->now() -
                  rclcpp::Duration::from_seconds(10);
      if (test == 2)
        m.model_context[2] = 0;
      if (test == 3)
        m.retained_parameters[0] = -1;
      if (test == 4)
        m.retained_parameters.resize(3);
      if (test == 5)
        m.measurements_fresh = false;
      if (test == 6)
        m.retained_accepted_time = 101;
      if (test == 7)
        m.observation_time = 90;
      h.receive_identified_model(m);
      h.apply_identified_models();
      EXPECT_EQ(h.identification_ack_[0], 0) << test;
    }
    auto m = message(h);
    h.receive_identified_model(m);
    h.apply_identified_models();
    ASSERT_EQ(h.identification_ack_[0], 1);
    h.identified_model_received_[0] -= std::chrono::seconds(4);
    h.apply_identified_models();
    EXPECT_EQ(h.identification_ack_[0], 0);
    m.stamp = h.node_frames_interface_->now();
    h.receive_identified_model(m);
    h.apply_identified_models();
    ASSERT_EQ(h.identification_ack_[0], 1);
    h.reset_identified_models();
    h.receive_identified_model(m);
    h.apply_identified_models();
    EXPECT_EQ(h.identification_ack_[0], 0);
  }
  static void gripper() {
    Hardware h;
    initialize(h);
    h.x_est_list_[4](0) = .01;
    h.x_est_list_[4](1) = .02;
    const auto plant_mass = h.endeffector_mass_;
    const auto P = h.P_est_list_[4];
    auto m = message(h, true);
    h.receive_identified_model(m);
    h.apply_identified_models();
    ASSERT_EQ(h.identification_ack_[3], 1);
    EXPECT_DOUBLE_EQ(h.gripper_observer_.x[0], .01);
    EXPECT_DOUBLE_EQ(h.gripper_observer_.x[1], .02);
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
        EXPECT_DOUBLE_EQ(h.gripper_observer_.covariance(i, j), double(P(i, j)));
    EXPECT_EQ(h.gripper_model_readback_[0], 320);
    EXPECT_EQ(h.endeffector_mass_, plant_mass);
    m.retained_valid = false;
    m.stamp = h.node_frames_interface_->now();
    h.receive_identified_model(m);
    h.apply_identified_models();
    EXPECT_EQ(h.identification_ack_[3], 0);
    EXPECT_DOUBLE_EQ(double(h.x_est_list_[4](0)), .01);
  }
};
} // namespace ros2_control_blue_reach_5
TEST(IdentifiedArmEKF, AtomicHandoverPreservesJointMarginalAndPlant) {
  rclcpp::init(0, nullptr);
  ros2_control_blue_reach_5::ArmEstimatorTestAccess::handover();
  rclcpp::shutdown();
}
TEST(IdentifiedArmEKF, RejectsStaleWrongGenerationAndMalformedModels) {
  rclcpp::init(0, nullptr);
  ros2_control_blue_reach_5::ArmEstimatorTestAccess::reject();
  rclcpp::shutdown();
}
TEST(IdentifiedArmEKF, GripperHandoverPreservesJointMarginalAndPlant) {
  rclcpp::init(0, nullptr);
  ros2_control_blue_reach_5::ArmEstimatorTestAccess::gripper();
  rclcpp::shutdown();
}
TEST(IdentifiedArmEKF, GripperPhysicsPredictionAndContacts) {
  using uvms::arm_identification::GripperObserver;
  GripperObserver filter;
  filter.handover(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity() * .001,
                  Eigen::Vector3d(2, 0, 0));
  // Constant 4-unit effort / mass 2 gives exactly q=.01, v=.2 after .1 s.
  filter.update(.1, 4, Eigen::Vector2d(.01, .2),
                Eigen::Matrix2d::Identity() * .01, false);
  EXPECT_NEAR(filter.x[0], .01, 1e-12);
  EXPECT_NEAR(filter.x[1], .2, 1e-12);
  EXPECT_NEAR(filter.acceleration(4), 2, 1e-12);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(filter.covariance);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0);
  filter.x[2] = .3;
  filter.update(.1, 4, Eigen::Vector2d(.02, 0),
                Eigen::Matrix2d::Identity() * .01, true);
  EXPECT_EQ(filter.x[0], .02);
  EXPECT_EQ(filter.x[1], 0);
  EXPECT_EQ(filter.x[2], .3);
}
