#include "ros2_control_blue_reach_5/vehicle_identifier.hpp"
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"
#include <gtest/gtest.h>
#include <random>

namespace vi = uvms::identification;
namespace {
vi::Window observations(double start, unsigned seed, const vi::Parameters & parameters) {
  vi::Window window{0, start, start + 10., Eigen::MatrixXd(606, 27), {}, {}, {}};
  std::mt19937 generator(seed);
  std::normal_distribution<double> normal;
  for (int row = 0; row < window.design.rows(); ++row)
    for (int column = 0; column < 27; ++column)
      window.design(row, column) = normal(generator);
  window.design.col(11) = -window.design.col(10);
  window.wrench = window.design * parameters;
  for (int i = 0; i <= 100; ++i) window.times.push_back(start + i * .1);
  return window;
}
}

// Requires the optional MOSEK SDK and license. Tests the real constrained solver,
// not sensor transport or the simulated plant.
TEST(IdentificationSolver, RetainsModelOnlyAfterIndependentPredictionPasses) {
  const auto nominal = ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  const vi::Parameters truth = Eigen::Map<const vi::Parameters>(nominal.data());
  vi::Identifier identifier(105., 150., 2.);
  identifier.preflight();
  vi::ModelRetention retention({}, false);
  auto solve = [&](const auto & a, const auto & b) { return identifier.fit(a, b); };
  const auto training = observations(0., 42, truth);
  auto result = retention.update(training, solve);
  ASSERT_TRUE(result.candidate) << result.status << ": " << result.detail;
  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.status, "awaiting_independent_validation");
  const auto validation = observations(11., 73, truth);
  result = retention.update(validation, solve);
  ASSERT_TRUE(result.accepted) << result.detail;
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.retained_net_weight_only);
  EXPECT_LT((validation.design * result.parameters - validation.wrench).norm(), .01);
  // W and B are inseparable underwater: test their difference, not each value.
  EXPECT_NEAR(result.parameters[10] - result.parameters[11], truth[10] - truth[11], .001);
  auto mismatch = observations(22., 91, truth);
  mismatch.wrench.array() += 10.;
  for (int i = 0; i < 3; ++i) {
    mismatch = observations(22. + 11. * i, 91 + i, truth);
    mismatch.wrench.array() += 10.;
    result = retention.update(mismatch, solve);
  }
  EXPECT_FALSE(result.valid);
}

TEST(IdentificationSolver, RobustFitReducesEffectOfSparseLargeOutliers) {
  const auto nominal = ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  const vi::Parameters truth = Eigen::Map<const vi::Parameters>(nominal.data());
  vi::Identifier identifier(105., 150., 2.);
  identifier.preflight();
  auto training = observations(0., 112, truth);
  for (int i = 0; i < training.wrench.size(); i += 30) training.wrench[i] += 100.;
  const auto plain = identifier.fit(training.design, training.wrench, false);
  const auto robust = identifier.fit(training.design, training.wrench, true);
  const auto validation = observations(11., 113, truth);
  EXPECT_LT((validation.design * robust.parameters - validation.wrench).norm(),
            .1 * (validation.design * plain.parameters - validation.wrench).norm());
}

#include "ros2_control_blue_reach_5/batch_uvms_core.hpp"
#include "ros2_control_blue_reach_5/identification_sample.hpp"

class IdentificationPlant : public testing::TestWithParam<int> {};
TEST_P(IdentificationPlant, NoisyMeasurementsPassIndependentValidation) {
  const auto nominal=ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  const vi::Parameters truth=Eigen::Map<const vi::Parameters>(nominal.data());
  vi::Identifier identifier(105.,150.,2.,vi::RetentionConfig().residual_scales);identifier.preflight();
  vi::ModelRetention retention({},false);
  vi::LiveWindow window(300,.25,4.);
  vi::HardwareSample sampler;
  std::mt19937 random(712);std::normal_distribution<double> noise(0.,.001);
  vi::RetentionResult result;
  bool accepted=false;
  double accepted_prediction=0.;
  ros2_control_blue_reach_5::BatchUvmsCore plant;
  plant.configure(1);
  std::vector<double> initial(22,0.);initial[2]=3.;
  initial[12]=3.1;initial[13]=.7;initial[14]=.4;initial[15]=2.1;initial[16]=.006;
  ASSERT_TRUE(plant.reset(false,initial));
  ASSERT_TRUE(plant.set_vehicle_params(nominal));
  for(int step=0;step<200*90;++step) {
    const double t=step*.005;
    const auto before=plant.observations();
    vi::Vector6 wrench;
    for(int j=0;j<6;++j) {
      wrench[j]=(j<3?20.:8.)*(std::sin((.31+.137*j)*t)+.4*std::sin((1.3+.17*j)*t));
      if(j>=3)wrench[j]-=30.*before[j]+10.*before[j+6];
    }
    wrench[2]+=truth[11]-truth[10];
    std::vector<double> actions(11,0.);
    for(int j=0;j<6;++j)actions[j]=wrench[j];
    ASSERT_TRUE(plant.set_actions(actions,step+1));plant.step(.005);
    const auto state=plant.observations();
    const vi::Vector6 velocity=Eigen::Map<const vi::Vector6>(state.data()+6);
    const Eigen::Vector3d angles=Eigen::Map<const Eigen::Vector3d>(state.data()+3);
    const double depth=state[2];
    std::array<double,6> applied,speeds;
    std::array<double,3> attitude;
    for(int j=0;j<6;++j)applied[j]=wrench[j];
    ASSERT_TRUE(sampler.advance(.005,applied));
    if(step%GetParam())continue;
    for(int j=0;j<6;++j)speeds[j]=velocity[j]+noise(random);
    for(int j=0;j<3;++j)attitude[j]=angles[j]+noise(random);
    ASSERT_TRUE(sampler.observe(depth,attitude,speeds));
    vi::Measurement measurement{sampler.generation,sampler.step,sampler.time,sampler.period,depth,
      Eigen::Map<Eigen::Vector3d>(attitude.data()),Eigen::Map<vi::Vector6>(speeds.data()),
      Eigen::Map<vi::Vector6>(sampler.wrench.data())};
    window.push(measurement);
    if(step%2000)continue;
    if(auto snapshot=window.snapshot()) {
      result=retention.update(*snapshot,[&](const auto& a,const auto& b){return identifier.fit(a,b);});
      if(result.accepted) {accepted=true;accepted_prediction=result.validation_error;}

    }
  }
  EXPECT_TRUE(accepted) << result.status << ": " << result.detail
    << " rank=" << result.information.rank << " error=" << result.validation_error;
  EXPECT_TRUE(result.valid);
  EXPECT_LE(accepted_prediction,1.);
  if(accepted)std::cout << "CPU plant with noisy " << 200/GetParam()
    << " Hz observations: validation score " << accepted_prediction << " <= 1" << std::endl;
}

INSTANTIATE_TEST_SUITE_P(SensorRates, IdentificationPlant, testing::Values(20, 1));

TEST(IdentificationSolver, PerAxisScalingIsInvariantToMeasurementUnits) {
  const auto nominal=ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  const vi::Parameters truth=Eigen::Map<const vi::Parameters>(nominal.data());
  auto data=observations(0.,197,truth);
  std::mt19937 random(49);std::normal_distribution<double> noise;
  const vi::Vector6 scales=vi::RetentionConfig().residual_scales;
  for(Eigen::Index i=0;i<data.wrench.size();++i)data.wrench[i]+=.2*scales[i%6]*noise(random);
  vi::Identifier original(105.,150.,2.,scales);
  const auto fit=original.fit(data.design,data.wrench);
  auto converted=data;vi::Vector6 converted_scales=scales;
  converted_scales.tail<3>()*=1000.;
  for(Eigen::Index i=0;i<data.wrench.size();++i)if(i%6>=3) {
    converted.design.row(i)*=1000.;converted.wrench[i]*=1000.;
  }
  vi::Identifier other_units(105.,150.,2.,converted_scales);
  const auto second=other_units.fit(converted.design,converted.wrench);
  EXPECT_LT((data.design*(fit.parameters-second.parameters)).norm(),1e-4);
  EXPECT_LT((fit.residual_rms.head<3>()-second.residual_rms.head<3>()).norm(),1e-6);
  EXPECT_LT((1000.*fit.residual_rms.tail<3>()-second.residual_rms.tail<3>()).norm(),1e-3);
}

class IdentificationPlantCoverage : public testing::TestWithParam<unsigned> {};
TEST_P(IdentificationPlantCoverage, PredictsReturnToHigherSpeeds) {
  const auto nominal=ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  const vi::Parameters truth=Eigen::Map<const vi::Parameters>(nominal.data());
  vi::Identifier identifier(105.,150.,2.,vi::RetentionConfig().residual_scales);identifier.preflight();
  vi::Identifier rolling_only(105.,150.,2.);
  vi::Parameters rolling_parameters;
  vi::Parameters archived_parameters;
  bool compared=false;
  vi::ModelRetention retention({},false);
  vi::LiveWindow window(300,.25,4.);
  vi::HardwareSample sampler;
  std::mt19937 random(GetParam());std::normal_distribution<double> noise(0.,.001);
  vi::RetentionResult result;
  bool accepted=false;
  double accepted_prediction=0.;
  ros2_control_blue_reach_5::BatchUvmsCore plant;
  plant.configure(1);
  std::vector<double> initial(22,0.);initial[2]=3.;
  initial[12]=3.1;initial[13]=.7;initial[14]=.4;initial[15]=2.1;initial[16]=.006;
  ASSERT_TRUE(plant.reset(false,initial));
  ASSERT_TRUE(plant.set_vehicle_params(nominal));
  for(int step=0;step<200*140;++step) {
    const double t=step*.005;
    const auto before=plant.observations();
    vi::Vector6 wrench;
    for(int j=0;j<6;++j) {
      wrench[j]=(j<3?(t<60. || t>=100. ? 60.:20.):8.)*(std::sin((.31+.137*j)*t)+.4*std::sin((1.3+.17*j)*t));
      if(j>=3)wrench[j]-=30.*before[j]+10.*before[j+6];
    }
    wrench[2]+=truth[11]-truth[10];
    std::vector<double> actions(11,0.);
    for(int j=0;j<6;++j)actions[j]=wrench[j];
    ASSERT_TRUE(plant.set_actions(actions,step+1));plant.step(.005);
    const auto state=plant.observations();
    const vi::Vector6 velocity=Eigen::Map<const vi::Vector6>(state.data()+6);
    const Eigen::Vector3d angles=Eigen::Map<const Eigen::Vector3d>(state.data()+3);
    const double depth=state[2];
    std::array<double,6> applied,speeds;
    std::array<double,3> attitude;
    for(int j=0;j<6;++j)applied[j]=wrench[j];
    ASSERT_TRUE(sampler.advance(.005,applied));
    if(step%20)continue;
    for(int j=0;j<6;++j)speeds[j]=velocity[j]+noise(random);
    for(int j=0;j<3;++j)attitude[j]=angles[j]+noise(random);
    ASSERT_TRUE(sampler.observe(depth,attitude,speeds));
    vi::Measurement measurement{sampler.generation,sampler.step,sampler.time,sampler.period,depth,
      Eigen::Map<Eigen::Vector3d>(attitude.data()),Eigen::Map<vi::Vector6>(speeds.data()),
      Eigen::Map<vi::Vector6>(sampler.wrench.data())};
    window.push(measurement);
    if(step%2000 && step!=19000 && step!=27000)continue;
    if(auto snapshot=window.snapshot()) {
      if(step==27000) {
        const auto rms=[&](const vi::Parameters & p) {
          vi::Vector6 sum=vi::Vector6::Zero();
          Eigen::VectorXd error=snapshot->design*p-snapshot->wrench;
          for(Eigen::Index i=0;i<error.size();++i)sum[i%6]+=error[i]*error[i];
          const vi::Vector6 normalized=(sum/(error.size()/6)).cwiseSqrt().cwiseQuotient(vi::RetentionConfig().residual_scales);
          return normalized.maxCoeff();
        };
        const double before=rms(rolling_parameters),after=rms(archived_parameters);
        std::cout << "Changing-speed prediction: rolling=" << before << " archived/scaled=" << after << " known-plant=" << rms(truth) << std::endl;
        EXPECT_LT(after,before);
        EXPECT_LE(after,1.);
        compared=true;
      }
      result=retention.update(*snapshot,[&](const auto& a,const auto& b){return identifier.fit(a,b);});
      if(result.accepted) {accepted=true;accepted_prediction=result.validation_error;}
      if(step==19000) {
        ASSERT_TRUE(result.candidate) << result.status;
        rolling_parameters=rolling_only.fit(snapshot->design,snapshot->wrench).parameters;
        archived_parameters=result.candidate->parameters;
      }

    }
  }
  EXPECT_TRUE(accepted) << result.status << ": " << result.detail
    << " rank=" << result.information.rank << " error=" << result.validation_error;
  EXPECT_TRUE(result.valid);
  EXPECT_LE(accepted_prediction,1.);
  EXPECT_TRUE(compared);
}

INSTANTIATE_TEST_SUITE_P(NoiseSeeds, IdentificationPlantCoverage, testing::Values(712u, 731u, 790u));

class RandomizedIdentificationPlant : public testing::TestWithParam<unsigned> {};
TEST_P(RandomizedIdentificationPlant, IndependentlyRandomizedDynamicsValidate) {
  auto nominal=ros2_control_blue_reach_5::default_nominal_vehicle_parameters();
  std::mt19937 parameter_rng(GetParam());
  std::uniform_real_distribution<double> unit(0.,1.);
  for(int j=0;j<3;++j)nominal[j]=20.+60.*unit(parameter_rng);
  for(int j=7;j<10;++j)nominal[j]=.4+3.6*unit(parameter_rng);
  nominal[3]=nominal[6]=(.6*unit(parameter_rng)-.3)*std::sqrt(nominal[0]*nominal[8]);
  nominal[4]=nominal[5]=(.6*unit(parameter_rng)-.3)*std::sqrt(nominal[1]*nominal[7]);
  nominal[10]=105.+45.*unit(parameter_rng);
  nominal[11]=nominal[10]-5.+10.*unit(parameter_rng);
  for(int j=12;j<15;++j)nominal[j]=-2.+4.*unit(parameter_rng);
  for(int j=15;j<21;++j)nominal[j]=j<18 ? -10.-90.*unit(parameter_rng) : -.2-6.*unit(parameter_rng);
  for(int j=21;j<27;++j)nominal[j]=j<24 ? -.1-80.*unit(parameter_rng) : -.1-4.*unit(parameter_rng);
  const vi::Parameters truth=Eigen::Map<const vi::Parameters>(nominal.data());
  vi::Identifier identifier(105.,150.,2.,vi::RetentionConfig().residual_scales);identifier.preflight();
  vi::ModelRetention retention({},false);
  vi::LiveWindow window(300,.25,4.);
  vi::HardwareSample sampler;
  std::mt19937 random(712);std::normal_distribution<double> noise(0.,.001);
  vi::RetentionResult result;
  bool accepted=false;
  double accepted_prediction=0.;
  ros2_control_blue_reach_5::BatchUvmsCore plant;
  plant.configure(1);
  std::vector<double> initial(22,0.);initial[2]=3.;
  initial[12]=3.1;initial[13]=.7;initial[14]=.4;initial[15]=2.1;initial[16]=.006;
  ASSERT_TRUE(plant.reset(false,initial));
  ASSERT_TRUE(plant.set_vehicle_params(nominal));
  for(int step=0;step<200*90;++step) {
    const double t=step*.005;
    const auto before=plant.observations();
    vi::Vector6 wrench;
    for(int j=0;j<6;++j) {
      wrench[j]=(j<3?20.:8.)*(std::sin((.31+.137*j)*t)+.4*std::sin((1.3+.17*j)*t));
      if(j>=3)wrench[j]-=30.*before[j]+10.*before[j+6];
    }
    wrench[2]+=truth[11]-truth[10];
    std::vector<double> actions(11,0.);
    for(int j=0;j<6;++j)actions[j]=wrench[j];
    ASSERT_TRUE(plant.set_actions(actions,step+1));plant.step(.005);
    const auto state=plant.observations();
    const vi::Vector6 velocity=Eigen::Map<const vi::Vector6>(state.data()+6);
    const Eigen::Vector3d angles=Eigen::Map<const Eigen::Vector3d>(state.data()+3);
    const double depth=state[2];
    std::array<double,6> applied,speeds;
    std::array<double,3> attitude;
    for(int j=0;j<6;++j)applied[j]=wrench[j];
    ASSERT_TRUE(sampler.advance(.005,applied));
    if(step%20)continue;
    for(int j=0;j<6;++j)speeds[j]=velocity[j]+noise(random);
    for(int j=0;j<3;++j)attitude[j]=angles[j]+noise(random);
    ASSERT_TRUE(sampler.observe(depth,attitude,speeds));
    vi::Measurement measurement{sampler.generation,sampler.step,sampler.time,sampler.period,depth,
      Eigen::Map<Eigen::Vector3d>(attitude.data()),Eigen::Map<vi::Vector6>(speeds.data()),
      Eigen::Map<vi::Vector6>(sampler.wrench.data())};
    window.push(measurement);
    if(step%2000)continue;
    if(auto snapshot=window.snapshot()) {
      result=retention.update(*snapshot,[&](const auto& a,const auto& b){return identifier.fit(a,b);});
      if(result.accepted) {accepted=true;accepted_prediction=result.validation_error;}

    }
  }
  EXPECT_TRUE(accepted) << result.status << ": " << result.detail
    << " rank=" << result.information.rank << " error=" << result.validation_error;
  EXPECT_TRUE(result.valid);
  EXPECT_LE(accepted_prediction,1.);
  if(accepted)std::cout << "CPU plant with noisy " << 10
    << " Hz observations: validation score " << accepted_prediction << " <= 1" << std::endl;
}

INSTANTIATE_TEST_SUITE_P(RandomSeeds, RandomizedIdentificationPlant, testing::Values(20261112u, 20261113u, 20261114u));
