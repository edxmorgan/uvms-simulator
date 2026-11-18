#include "ros2_control_blue_reach_5/arm_identifier.hpp"
#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"
#include <gtest/gtest.h>
#include <random>
using namespace uvms::arm_identification;
using namespace ros2_control_blue_reach_5;
using casadi::DM;
namespace {
Eigen::VectorXd parameters(const msg::SimManipulatorDynamics &p) {
  Eigen::VectorXd t(48);
  for (int j = 0; j < 4; ++j) {
    t[12 * j] = p.link_masses[j];
    for (int k = 0; k < 3; ++k)
      t[12 * j + 1 + k] = p.link_first_moments[3 * j + k];
    for (int k = 0; k < 6; ++k)
      t[12 * j + 4 + k] = p.link_inertias[6 * j + k];
    t[12 * j + 10] = p.viscous_friction[j];
    t[12 * j + 11] = p.coulomb_friction[j];
  }
  return t;
}
std::vector<Row> trajectory(double seconds = 65, double noise = 0,
                            int seed = 72, bool random_plant = false) {
  auto p = default_manipulator_dynamics();
  // Unknown plant differs from nominal. No plant parameters are passed to
  // Identifier.
  for (auto &v : p.link_inertias)
    v *= 1.15;
  for (auto &v : p.viscous_friction)
    v *= .8;
  if (random_plant) {
    std::mt19937 random(seed);
    std::uniform_real_distribution<double> unit(0., 1.);
    for (int j = 0; j < 4; ++j) {
      const double mass = .08 + 1.12 * unit(random);
      Eigen::Vector3d com;
      Eigen::Matrix3d rotation;
      for (int k = 0; k < 3; ++k)
        com[k] = -.08 + .16 * unit(random);
      for (int k = 0; k < 9; ++k)
        rotation.data()[k] = unit(random) - .5;
      rotation = Eigen::HouseholderQR<Eigen::Matrix3d>(rotation).householderQ();
      Eigen::Vector3d moments;
      for (int k = 0; k < 3; ++k)
        moments[k] = .003 + .047 * unit(random);
      const Eigen::Matrix3d second =
          rotation * moments.asDiagonal() * rotation.transpose();
      const Eigen::Matrix3d inertia =
          second.trace() * Eigen::Matrix3d::Identity() - second +
          mass * (com.squaredNorm() * Eigen::Matrix3d::Identity() -
                  com * com.transpose());
      p.link_masses[j] = mass;
      for (int k = 0; k < 3; ++k)
        p.link_first_moments[3 * j + k] = mass * com[k];
      const double values[] = {inertia(0, 0), inertia(1, 1), inertia(2, 2),
                               inertia(0, 1), inertia(0, 2), inertia(1, 2)};
      for (int k = 0; k < 6; ++k)
        p.link_inertias[6 * j + k] = values[k];
      p.viscous_friction[j] = .15 + 2.85 * unit(random);
      p.coulomb_friction[j] = .02 + .18 * unit(random);
    }
  }
  const auto theta = parameters(p);
  auto plant =
      casadi::external("arm_plant_next", std::string(TEST_CASADI_DIR) +
                                             "/libmDisturbanceEKF_next.so");
  auto packed = DM::vertcat(pack_manipulator_dynamics(p));
  DM state = DM::zeros(10), torque = DM::zeros(5);
  Sample sample;
  sample.supported = 1;
  Window window;
  std::mt19937 rng(seed);
  std::normal_distribution<double> n(0, noise);
  std::vector<Row> rows;
  const double dt = .005;
  for (int i = 0; i < seconds / dt; ++i) {
    double time = i * dt;
    Eigen::Vector4d acc;
    for (int j = 0; j < 4; ++j) {
      const double f = .37 + .13 * j, h = 1.07 + .17 * j;
      double q = .35 * std::sin(f * time) + .12 * std::sin(h * time);
      double v = .35 * f * std::cos(f * time) + .12 * h * std::cos(h * time);
      double a =
          -.35 * f * f * std::sin(f * time) - .12 * h * h * std::sin(h * time);
      sample.position[j] = double(state(j));
      sample.velocity[j] = double(state(5 + j));
      acc[j] = a + 10 * (q - sample.position[j]) + 5 * (v - sample.velocity[j]);
    }
    Eigen::Vector4d tau = regressor(sample, acc) * theta;
    for (int j = 0; j < 4; ++j)
      torque(j) = tau[j];
    double q = .01 * std::sin(.7 * time) + .004 * std::sin(1.3 * time);
    double v = .007 * std::cos(.7 * time) + .0052 * std::cos(1.3 * time);
    double a = -.0049 * std::sin(.7 * time) - .00676 * std::sin(1.3 * time);
    torque(4) =
        320 * (a + 10 * (q - double(state(4))) + 5 * (v - double(state(9)))) +
        380 * double(state(9)) + 20 * double(state(4));
    state = plant(std::vector<DM>{state, torque, dt, packed, 320, 380, 20,
                                  DM::zeros(4), 200})[0];
    std::array<double, 5> qs{}, vs{}, ts{};
    std::array<bool, 5> usable{true, true, true, true, true};
    for (int j = 0; j < 5; ++j) {
      qs[j] = double(state(j));
      vs[j] = double(state(5 + j));
      ts[j] = double(torque(j));
    }
    sample.advance(dt, qs, vs, ts, usable);
    if (i % 4 == 3) {
      Sample measured = sample;
      for (int j = 0; j < 5; ++j) {
        measured.position[j] += n(rng);
        measured.velocity[j] += n(rng);
      }
      auto block = window.observe(measured);
      rows.insert(rows.end(), block.begin(), block.end());
    }
  }
  return rows;
}
} // namespace
TEST(ArmIdentification, RegressorMatchesIndependentPlantAcceleration) {
  auto p = default_manipulator_dynamics();
  auto theta = parameters(p);
  auto plant =
      casadi::external("arm_plant_next", std::string(TEST_CASADI_DIR) +
                                             "/libmDisturbanceEKF_next.so");
  Sample s;
  Eigen::Vector4d a;
  DM state = DM::zeros(10), input = DM::zeros(5);
  for (int j = 0; j < 4; ++j) {
    s.position[j] = .2 * (j - 1);
    s.velocity[j] = .1 * (j + 1);
    a[j] = .3 - .1 * j;
    state(j) = s.position[j];
    state(5 + j) = s.velocity[j];
  }
  Eigen::Vector4d tau = regressor(s, a) * theta;
  for (int j = 0; j < 4; ++j)
    input(j) = tau[j];
  auto next = plant(std::vector<DM>{state, input, 1e-6,
                                    DM::vertcat(pack_manipulator_dynamics(p)),
                                    300, 400, 0, DM::zeros(4), 200})[0];
  for (int j = 0; j < 4; ++j)
    EXPECT_NEAR((double(next(5 + j)) - s.velocity[j]) / 1e-6, a[j], 1e-4);
}
TEST(ArmIdentification, ImpulseAndInvalidIntervals) {
  Sample s;
  s.supported = 1;
  std::array<double, 5> q{}, v{}, tau{1, 2, 3, 4, 5};
  std::array<bool, 5> valid{true, true, true, true, true};
  s.advance(.01, q, v, tau, valid);
  tau[0] = 3;
  s.advance(.02, q, v, tau, valid);
  EXPECT_NEAR(s.impulse[0], .07, 1e-12);
  valid[1] = false;
  s.advance(.01, q, v, tau, valid);
  EXPECT_EQ(s.generation[1], 1);
  EXPECT_EQ(s.impulse[1], 0);
  s.reset();
  EXPECT_EQ(s.time, 0);
  EXPECT_EQ(s.valid[0], 0);
}
TEST(ArmIdentification, WeakRowsMatchActualPlantAndDoNotBridgeLocksOrGaps) {
  auto rows = trajectory(8);
  auto p = default_manipulator_dynamics();
  for (auto &v : p.link_inertias)
    v *= 1.15;
  for (auto &v : p.viscous_friction)
    v *= .8;
  auto theta = parameters(p);
  double worst = 0;
  ASSERT_FALSE(rows.empty());
  for (const auto &r : rows)
    if (r.axis < 4)
      worst = std::max(worst, std::abs((r.design * theta)[0] - r.torque));
  EXPECT_LT(worst, .003);
  Window w;
  Sample s;
  s.supported = 1;
  s.valid.fill(1);
  for (int i = 0; i < 150; ++i) {
    s.time = i * .02;
    w.observe(s);
  }
  s.time += 1;
  EXPECT_TRUE(w.observe(s).empty());
  s.generation[0]++;
  s.time += .02;
  EXPECT_TRUE(w.observe(s).empty());
}
TEST(ArmIdentification, QuietDataCannotBecomeValidated) {
  Identifier id;
  Window window;
  Sample s;
  s.supported = 1;
  s.valid.fill(1);
  for (int i = 0; i < 2500; ++i) {
    s.time = i * .02;
    id.observe(window.observe(s));
  }
  EXPECT_FALSE(id.result().valid);
  EXPECT_LT(id.result().rank, id.result().dimensions);
}
class ArmIdentificationPlant : public ::testing::TestWithParam<int> {};
TEST_P(ArmIdentificationPlant,
       PhysicalFitAndIndependentValidationOnChangedPlant) {
  const bool randomized = GetParam() > 1000;
  auto rows = trajectory(randomized ? 180 : 85, 1e-4, GetParam(), randomized);
  Identifier arm;
  Config c;
  c.gripper = true;
  c.horizon = 300;
  c.residual_scale = .1;
  Identifier gripper(c);
  bool accepted_arm = false, accepted_gripper = false;
  double arm_accepted_error = -1, gripper_accepted_error = -1;
  for (size_t i = 0; i < rows.size();) {
    std::vector<Row> block;
    double time = rows[i].end;
    while (i < rows.size() && rows[i].end == time)
      block.push_back(rows[i++]);
    arm.observe(block);
    gripper.observe(block);
    accepted_arm |= arm.result().valid;
    accepted_gripper |= gripper.result().valid;
    if (arm.result().status == "validated")
      arm_accepted_error = arm.result().validation_error;
    if (gripper.result().status == "validated")
      gripper_accepted_error = gripper.result().validation_error;
  }
  std::cout << "arm status=" << arm.result().status
            << " rank=" << arm.result().rank << "/" << arm.result().dimensions
            << " accepted error=" << arm_accepted_error
            << " detail=" << arm.result().detail << "\n";
  std::cout << "gripper status=" << gripper.result().status
            << " accepted error=" << gripper_accepted_error
            << " detail=" << gripper.result().detail << "\n";
  EXPECT_TRUE(accepted_arm);
  EXPECT_TRUE(accepted_gripper);
  if (accepted_gripper) {
    EXPECT_NEAR(gripper.result().retained[0], 320, 5);
    EXPECT_NEAR(gripper.result().retained[1], 380, 5);
    EXPECT_NEAR(gripper.result().retained[2], 20, 5);
  }
  arm.reset();
  EXPECT_FALSE(arm.result().valid);
  EXPECT_EQ(arm.result().rows, 0);
}

TEST(ArmIdentification, IndependentRejectionKeepsModelUntilPersistentMismatch) {
  Config config;
  config.horizon = 240;
  config.validation_seconds = 2.;
  Identifier id(config);
  auto theta = parameters(default_manipulator_dynamics());
  std::mt19937 rng(81);
  std::uniform_real_distribution<double> u(-1, 1);
  bool accepted = false, rejected = false, invalidated = false;
  for (int i = 0; i < 400; ++i) {
    Sample s;
    Eigen::Vector4d acc;
    for (int j = 0; j < 4; ++j) {
      s.position[j] = u(rng);
      s.velocity[j] = u(rng);
      acc[j] = u(rng);
    }
    auto a = regressor(s, acc);
    Eigen::Vector4d torque = a * theta;
    std::vector<Row> rows;
    for (int j = 0; j < 4; ++j) {
      Row r;
      r.axis = j;
      r.start = i * .1;
      r.end = r.start + .05;
      r.design = a.row(j);
      r.torque = torque[j] + (i > 160 ? 2 * u(rng) : 0);
      rows.push_back(r);
    }
    id.observe(rows);
    accepted |= id.result().valid;
    rejected |= id.result().valid && id.result().validation_error > 1;
    invalidated |= accepted && !id.result().valid;
  }
  EXPECT_TRUE(accepted);
  EXPECT_TRUE(rejected);
  EXPECT_TRUE(invalidated);
}

TEST(ArmIdentification, ValidationCannotReuseTrainingSupport) {
  Config c;
  c.gripper = true;
  c.horizon = 60;
  c.validation_seconds = 1.;
  Identifier id(c);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1, 1);
  const Eigen::Vector3d p(300, 400, 20);
  for (int i = 0; i < 60; ++i) {
    Row r;
    r.axis = 4;
    r.start = i * .1;
    r.end = r.start + 2;
    r.design = Eigen::RowVector3d(u(rng), u(rng), u(rng));
    r.torque = (r.design * p)[0];
    id.observe({r});
  }
  ASSERT_EQ(id.result().status, "checking independent measurements");
  const auto candidate = id.result().candidate;
  for (int i = 0; i < 30; ++i) {
    Row r;
    r.axis = 4;
    r.start = 5.8;
    r.end = 8. + .01 * i;
    r.design = Eigen::RowVector3d(u(rng), u(rng), u(rng));
    r.torque = (r.design * p)[0];
    id.observe({r});
  }
  EXPECT_FALSE(id.result().valid);
  EXPECT_EQ(id.result().validation_error, -1);
  EXPECT_EQ((id.result().candidate - candidate).norm(), 0);
}

INSTANTIATE_TEST_SUITE_P(NoiseSeeds, ArmIdentificationPlant,
                         ::testing::Values(72, 111, 194, 20261112, 20261113,
                                           20261114));

TEST(ArmIdentification, StickingJointCannotSupplySlidingFrictionRows) {
  Window window;
  Sample sample;
  sample.supported = 1;
  sample.valid.fill(1);
  int counts[5] = {};
  for (int i = 0; i < 250; ++i) {
    sample.time = i * .02;
    sample.velocity.fill(.1);
    sample.velocity[3] = (i % 2 ? 1 : -1) * .0005;
    for (const auto &row : window.observe(sample))
      ++counts[row.axis];
  }
  EXPECT_EQ(counts[3], 0);
  EXPECT_GT(counts[0], 0);
  EXPECT_GT(counts[4], 0);
}

TEST(ArmIdentification, CollectionReportsActualPerAxisRowsAndResets) {
  Identifier id;
  std::vector<Row> rows;
  for (int axis : {0, 2, 2, 3, 4}) {
    Row row;
    row.axis = axis;
    row.design = Eigen::RowVectorXd::Zero(axis == 4 ? 3 : 48);
    row.start = 0.; row.end = 2.;
    rows.push_back(row);
  }
  id.observe(rows);
  EXPECT_EQ(id.result().status, "collecting");
  EXPECT_EQ(id.result().rows, 4);
  EXPECT_EQ(id.result().rows_by_axis, (std::array<int, 5>{1, 0, 2, 1, 0}));
  EXPECT_FALSE(id.result().valid);
  id.reset();
  EXPECT_EQ(id.result().rows_by_axis, (std::array<int, 5>{}));
}
