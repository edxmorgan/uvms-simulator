#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/nominal_vehicle_parameters.hpp"
#include "ros2_control_blue_reach_5/vehicle_disturbance_noise_adaptation.hpp"
using namespace ros2_control_blue_reach_5;
using Adaptation=VehicleDisturbanceNoiseAdaptation;
using casadi::DM;
namespace {
Adaptation::Snapshot sample(uint64_t step,double dt=.005) {
  Adaptation::Snapshot s; s.step=step; s.dt=dt; for(size_t i=0;i<6;++i) s.applied_covariance[7*i]=.0007;
  s.prior[0]=dt;
  for(size_t i=0;i<18;++i) {s.filtered_covariance[19*i]=1.;s.prior_covariance[19*i]=1.;s.transition[19*i]=1.;}
  return s;
}
void known_moment(const Adaptation::Snapshot& a,const Adaptation::Snapshot& b,
                  const std::array<double,18>&,const std::array<double,324>&,
                  std::array<double,18>& x,std::array<double,324>& P,Adaptation::Covariance& moments) {
  x=a.filtered; P=a.filtered_covariance; moments.fill(0.); for(size_t i=0;i<6;++i) moments[7*i]=b.dt*.2;
}
bool feed(Adaptation& adapter,const Adaptation::Snapshot& s) {
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!adapter.submit(s)) {
    if(std::chrono::steady_clock::now()>until)return false;
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  return true;
}
std::optional<Adaptation::Result> await_result(Adaptation& adapter) {
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<until) {
    if(auto result=adapter.poll())return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return std::nullopt;
}
}
TEST(VehicleDisturbanceNoiseAdaptation, PastWindowKnownMomentsAndConfiguredShrinkage) {
  Adaptation adapter(known_moment,{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=400;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(adapter.poll()); // first eligible fit at2.005s, same sample convention
  ASSERT_TRUE(feed(adapter,sample(401)));
  auto result=await_result(adapter); ASSERT_TRUE(result);
  EXPECT_EQ(result->endpoint_step,401u); EXPECT_EQ(result->transitions,399u);
  EXPECT_NEAR(result->window_seconds,1.995,1e-10);
  for(double q:result->density) EXPECT_NEAR(q,.5*.0007+.5*(.2+.1*.0007)/1.1,1e-12);
  EXPECT_GT(result->fit_seconds,0.); EXPECT_EQ(adapter.failed(),0u);
}
TEST(VehicleDisturbanceNoiseAdaptation, VariableDtUsesIndividualIncrementRates) {
  Adaptation adapter(known_moment,{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=136;++i) ASSERT_TRUE(feed(adapter,sample(i,i%2?.01:.02)));
  auto result=await_result(adapter); ASSERT_TRUE(result);
  for(double q:result->density) EXPECT_NEAR(q,.5*.0007+.5*(.2+.1*.0007)/1.1,1e-12);
}
TEST(VehicleDisturbanceNoiseAdaptation, StepGapStartsFreshWindow) {
  Adaptation adapter(known_moment,{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=300;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  for(uint64_t i=302;i<=500;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(adapter.poll());
  for(uint64_t i=501;i<=702;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  auto result=await_result(adapter); ASSERT_TRUE(result); EXPECT_EQ(result->endpoint_step,702u);
}
TEST(VehicleDisturbanceNoiseAdaptation, ClearRejectsInflightFitAndSubmitDoesNotWait) {
  std::atomic<bool> entered{false},release{false};
  Adaptation adapter([&](const auto& a,const auto& b,const auto& xs,const auto& Ps,auto& x,auto& P,auto& moments) {
    entered=true;
    while(!release.load()) std::this_thread::sleep_for(std::chrono::microseconds(100));
    known_moment(a,b,xs,Ps,x,P,moments);
  },{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=401;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!entered && std::chrono::steady_clock::now()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // Release before any fatal assertion to guarantee safe test teardown.
  if(!entered) {release=true;FAIL()<<"worker did not begin";}
  adapter.clear();
  const auto start=std::chrono::steady_clock::now();
  for(uint64_t i=402;i<=602;++i) adapter.submit(sample(i));
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  release=true;
  EXPECT_LT(seconds,.1); EXPECT_GT(adapter.dropped(),0u);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(adapter.poll()); EXPECT_EQ(adapter.failed(),0u);
}
TEST(VehicleDisturbanceNoiseAdaptation, RejectsNonfiniteMomentsWithoutPublishing) {
  Adaptation adapter([](const auto& a,const auto&,const auto&,const auto&,auto& x,auto& P,auto& moments) {
    x=a.filtered; P=a.filtered_covariance; moments.fill(std::numeric_limits<double>::quiet_NaN());
  },{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=401;++i) ASSERT_TRUE(feed(adapter,sample(i)));
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!adapter.failed() && std::chrono::steady_clock::now()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(adapter.failed(),1u); EXPECT_FALSE(adapter.poll());
}
TEST(VehicleDisturbanceNoiseAdaptation, GeneratedHelperRandomWalkFixedPointAndCost) {
  const char* library=std::getenv("UVMS_TEST_RTS_HELPER");
  if(!library) GTEST_SKIP()<<"Set UVMS_TEST_RTS_HELPER to generated native library for export parity check";
  Adaptation adapter(std::string(library),{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=401;++i) {
    auto s=sample(i); s.prior.fill(0.);
    // No observations: exact random-walk prediction/posterior, fixed diagonal
    // density. The conditional increment moment must preserve configured Q.
    for(size_t j=12;j<18;++j) {
      s.filtered_covariance[19*j]=s.prior_covariance[19*j]=.001+.0007*s.dt*double(i);
    }
    ASSERT_TRUE(feed(adapter,s));
  }
  auto result=await_result(adapter); ASSERT_TRUE(result);
  for(double q:result->density) EXPECT_NEAR(q,.0007,1e-11);
  EXPECT_EQ(result->transitions,399u);
  std::cout<<"Generated RTS400-sample window fit_seconds="<<result->fit_seconds<<std::endl;
}
TEST(VehicleDisturbanceNoiseBinding, DisabledPreservesFullCovarianceAndZeroConfiguration) {
  VehicleDisturbanceNoiseBinding b;
  b.configure({0.,.01,.02,.03,.04,.05},std::string());
  auto Q=DM::eye(18); Q(12,13)=Q(13,12)=.1; auto expected=Q;
  b.reset(Q); b.before_prediction(Q,true); b.after_correction({}, {}, .005,1);
  EXPECT_EQ(double(DM::norm_inf(Q-expected)),0.);
  EXPECT_EQ(b.active_density[0],0.); EXPECT_EQ(b.active_density[5],.05);
  EXPECT_EQ(b.diagnostics[0],0.); b.stop();
}
TEST(VehicleDisturbanceNoiseBinding, AppliesFitsAndRestoresOnUnavailableOrMalformedSnapshot) {
  const char* library=std::getenv("UVMS_TEST_RTS_HELPER");
  if(!library) GTEST_SKIP()<<"Generated helper library required";
  VehicleDisturbanceNoiseBinding b; b.configure({.0007,.0007,.0007,.0007,.0007,.0007},library);
  auto Q=DM::zeros(18,18); b.reset(Q);
  for(uint64_t i=1;i<=401;++i) {
    b.before_prediction(Q,true);
    b.after_correction({DM::zeros(18,1),DM::eye(18),DM::eye(18)},
                       {DM::zeros(18,1),DM::eye(18)},.005,i);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(b.diagnostics[1]==0. && std::chrono::steady_clock::now()<until) {
    b.before_prediction(Q,true); std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(b.diagnostics[1],1.);
  EXPECT_NEAR(double(Q(12,12)),.5*.0007+.5*(.1*.0007)/1.1,1e-12);
  EXPECT_EQ(b.diagnostics[5],401.); EXPECT_GT(b.diagnostics[4],0.);
  b.before_prediction(Q,false);
  EXPECT_DOUBLE_EQ(double(Q(12,12)),.0007); EXPECT_EQ(b.diagnostics[1],0.);
  // A malformed snapshot cannot leave a learned intensity active on the next read.
  b.before_prediction(Q,true); Q(12,12)=.5;
  b.after_correction({}, {}, .005,402);
  b.before_prediction(Q,true);
  EXPECT_DOUBLE_EQ(double(Q(12,12)),.0007); EXPECT_GE(b.diagnostics[3],1.);
  b.stop(); EXPECT_EQ(b.diagnostics[0],0.);
}

TEST(VehicleDisturbanceNoiseAdaptation, RawWorkerConcurrentWithNativeCasadiPrediction) {
  const char* library=std::getenv("UVMS_TEST_RTS_HELPER");
  if(!library) GTEST_SKIP()<<"Generated helper library required";
  auto predict=casadi::external("disturbance_ekf_predict_covariance",library);
  const DM parameters(default_nominal_vehicle_parameters());
  const std::vector<DM> args{DM::zeros(18,1),DM::eye(18),DM::zeros(6,1),parameters,.005,DM::eye(18)*.0007};
  const auto expected=predict(args);
  Adaptation worker(std::string(library),{.0007,.0007,.0007,.0007,.0007,.0007});
  for(int repetition=0;repetition<20;++repetition) {
    worker.clear();
    for(uint64_t i=1;i<=401;++i) {
      auto s=sample(i);s.prior.fill(0.);
      for(size_t j=12;j<18;++j)
        s.filtered_covariance[19*j]=s.prior_covariance[19*j]=.001+.0007*s.dt*double(i);
      ASSERT_TRUE(feed(worker,s));
    }
    // The controller-side CasADi prediction executes while the raw worker fits.
    for(int call=0;call<500;++call) {
      const auto actual=predict(args);
      ASSERT_EQ(actual.size(),expected.size());
      for(size_t j=0;j<actual.size();++j)
        ASSERT_EQ(double(DM::norm_inf(actual[j]-expected[j])),0.);
    }
    auto result=await_result(worker);ASSERT_TRUE(result);
    for(double q:result->density) EXPECT_NEAR(q,.0007,1e-11);
  }
  EXPECT_EQ(worker.failed(),0u);
}
TEST(VehicleDisturbanceNoiseAdaptation, FullCorrelatedRandomWalkAndRotationEquivariance) {
  const char* library=std::getenv("UVMS_TEST_RTS_HELPER");
  if(!library) GTEST_SKIP()<<"Generated full moment helper library required";
  using Matrix=Eigen::Matrix<double,6,6,Eigen::ColMajor>;
  Matrix covariance=Matrix::Identity()*.0007;
  covariance(0,4)=covariance(4,0)=.0002;
  covariance(1,5)=covariance(5,1)=-.0003;
  Matrix rotation=Matrix::Zero();
  const double c=std::cos(.73),s=std::sin(.73);
  Eigen::Matrix3d R; R<<c,-s,0,s,c,0,0,0,1;
  rotation.block<3,3>(0,0)=R; rotation.block<3,3>(3,3)=R;
  Matrix observed[2];
  for(int trial=0;trial<2;++trial) {
    const Matrix C=trial==0?covariance:Matrix(rotation*covariance*rotation.transpose());
    Adaptation worker(std::string(library),{.0007,.0007,.0007,.0007,.0007,.0007});
    for(uint64_t i=1;i<=401;++i) {
      auto snapshot=sample(i);snapshot.prior.fill(0.);
      for(size_t col=0;col<6;++col) for(size_t row=0;row<6;++row) {
        const auto k=(12+row)+18*(12+col);
        snapshot.applied_covariance[row+6*col]=C(row,col);
        snapshot.filtered_covariance[k]=snapshot.prior_covariance[k]=
            (row==col?.001:0.)+C(row,col)*snapshot.dt*double(i);
      }
      ASSERT_TRUE(feed(worker,snapshot));
    }
    auto result=await_result(worker);ASSERT_TRUE(result);
    observed[trial]=Eigen::Map<const Matrix>(result->covariance.data());
    const Matrix expected=.5*C+.5*(C+.1*.0007*Matrix::Identity())/1.1;
    EXPECT_LT((observed[trial]-expected).cwiseAbs().maxCoeff(),1e-11);
  }
  EXPECT_LT((observed[1]-rotation*observed[0]*rotation.transpose()).cwiseAbs().maxCoeff(),1e-11);
  EXPECT_GT(std::abs(observed[0](0,4)),.0001); // correlations must survive
}
TEST(VehicleDisturbanceNoiseAdaptation, RejectsMaterialNegativeMoments) {
  Adaptation worker([](const auto& a,const auto&,const auto&,const auto&,auto& x,auto& P,auto& moments) {
    x=a.filtered;P=a.filtered_covariance;moments.fill(0.);moments[0]=-.01;
  },{.0007,.0007,.0007,.0007,.0007,.0007});
  for(uint64_t i=1;i<=401;++i) ASSERT_TRUE(feed(worker,sample(i)));
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!worker.failed() && std::chrono::steady_clock::now()<until) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(worker.failed(),1u);EXPECT_FALSE(worker.poll());
}
