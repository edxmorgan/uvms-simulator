#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/hardware_arm_observer.hpp"
using namespace ros2_control_blue_reach_5;
using casadi::DM;
struct HardwareObserverTest : testing::Test {
  HardwareArmObserver h;
  std::array<JointFeedbackClock,4> clocks;
  HardwareArmObserver::Clock::time_point now=HardwareArmObserver::Clock::now();
  DM y=DM(std::vector<double>{3.1,2.1,1.0,1.0,0.,0.,0.,0.});
  DM u=DM::zeros(4,1), lower=DM::zeros(4,1), upper=DM::ones(4,1)*6.;
  void SetUp() override {
    h.predict=casadi::external("arm_disturbance_ekf_predict_locked",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
    h.update=casadi::external("arm_disturbance_ekf_update_limited",TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
    h.reset();
  }
  void tick() {
    now+=std::chrono::milliseconds(50);
    for(auto &c:clocks) for(int j=0;j<3;++j) c.update(j,now);
  }
  bool observe() { return h.observe(clocks,y,u,lower,upper,now); }
};
TEST_F(HardwareObserverTest, SameNativePosteriorAndMeasuredInput) {
  tick(); EXPECT_FALSE(observe());
  u=DM(std::vector<double>{.03,-.02,.01,.04});
  auto expected=arm_time_integration::posterior(h.predict,h.update,std::vector<DM>{h.x,h.P,u*.5,h.parameters,.05,y,h.Q*.05,h.R,
    DM::zeros(4,1),200.,lower,upper,DM::zeros(4,1)});
  tick(); ASSERT_TRUE(observe());
  EXPECT_LT(double(DM::norm_inf(h.x-expected[0])),1e-12);
  EXPECT_LT(double(DM::norm_inf(h.P-expected[1])),1e-12);
  EXPECT_EQ(h.valid,1); EXPECT_EQ(h.step,1);
  for(auto v:h.admitted) EXPECT_EQ(v,0); // NEVER enable physical compensation
  auto P=h.P; EXPECT_FALSE(observe());
  EXPECT_EQ(h.step,1); EXPECT_EQ(double(DM::norm_inf(h.P-P)),0);
}
TEST_F(HardwareObserverTest, MissingStaleAndPartiallyUpdatedPackets) {
  EXPECT_FALSE(observe()); EXPECT_EQ(h.valid,0);
  tick(); observe(); tick(); ASSERT_TRUE(observe());
  now+=std::chrono::milliseconds(20);
  clocks[0].update(0,now);
  EXPECT_FALSE(observe()); EXPECT_EQ(h.step,1);
  now+=std::chrono::milliseconds(151);
  EXPECT_FALSE(observe()); EXPECT_EQ(h.valid,0); EXPECT_FALSE(h.seeded);
  tick(); EXPECT_FALSE(observe()); // re-seed after dropout, no long prediction
  tick(); EXPECT_TRUE(observe());
}
TEST_F(HardwareObserverTest, RejectsSkewFutureAndNonfiniteFeedback) {
  tick(); clocks[0].received[0]-=std::chrono::milliseconds(60);
  EXPECT_FALSE(observe()); EXPECT_FALSE(h.seeded);
  tick(); clocks[0].received[0]+=std::chrono::milliseconds(1);
  EXPECT_FALSE(observe());
  tick(); y(0)=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(observe()); EXPECT_EQ(h.valid,0);
}
TEST_F(HardwareObserverTest, MechanicalStopIsNotSimulatedContactTruth) {
  tick(); observe(); tick(); ASSERT_TRUE(observe());
  tick(); y(3)=.005;
  EXPECT_FALSE(observe()); EXPECT_EQ(h.at_limit[3],1); EXPECT_EQ(h.valid,0);
  tick(); y(3)=1.; EXPECT_FALSE(observe()); // reseed release
  tick(); EXPECT_TRUE(observe()); EXPECT_EQ(h.at_limit[3],0);
  auto generation=h.generation; h.reset();
  EXPECT_EQ(h.generation,generation+1); EXPECT_EQ(h.step,0); EXPECT_EQ(h.valid,0);
}
TEST_F(HardwareObserverTest, NumericalFailureCannotLeakValidPosterior) {
  tick(); observe(); h.update=casadi::Function(); tick();
  EXPECT_FALSE(observe()); EXPECT_EQ(h.valid,0); EXPECT_FALSE(h.seeded);
}
TEST_F(HardwareObserverTest, RecoversSignedLoadsAtSerialPollingRate) {
  auto predict=casadi::external("arm_disturbance_ekf_predict_limited",
    TEST_CASADI_DIR "/libmDisturbanceEKF_next.so");
  // Numerical free-motion fixture, not physical joint limits or a hardware run.
  lower=DM::ones(4,1)*-100.; upper=DM::ones(4,1)*100.;
  DM truth=DM::zeros(12,1);
  for(int i=0;i<8;++i) truth(i)=y(i);
  const std::array<double,4> load{.03,-.02,.01,-.04};
  for(int i=0;i<4;++i) truth(8+i)=load[i];
  tick(); observe();
  for(int k=0;k<100;++k) {
    truth=predict(std::vector<DM>{truth,u,h.parameters,.05,DM::zeros(4,1),200.,lower,upper})[0];
    for(int i=0;i<8;++i) y(i)=truth(i);
    tick(); ASSERT_TRUE(observe());
  }
  for(int i=0;i<4;++i) EXPECT_NEAR(h.torque[i],load[i],.005);
}
