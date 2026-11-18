#include <gtest/gtest.h>
#include <future>
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
  for(int i=0;i<4;++i) for(int j=0;j<4;++j)
    EXPECT_DOUBLE_EQ(h.covariance[4*i+j],double(h.P(8+i,8+j)));
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
  for(int i=0;i<4;++i) for(int j=0;j<4;++j)
    EXPECT_DOUBLE_EQ(h.covariance[4*i+j],i==j?1.:0.);
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

TEST(HardwareKinematicFilter, PredictsWithoutReusingPacketsAndAcceptsPartialUpdates)
{
  auto f=casadi::external("ekf_update_fresh",TEST_CASADI_DIR "/libmEKF_next.so");
  auto old=casadi::external("ekf_update",TEST_CASADI_DIR "/libmEKF_next.so");
  DM x=DM(std::vector<double>{1.,.2,.1}), P=DM::eye(3)*.001;
  DM Q=DM::eye(3)*.001, R=DM::eye(2)*.01;
  DM y=DM(std::vector<double>{1.1,.3});
  const double dt=.005;
  DM T=DM::eye(3);T(0,1)=dt;T(0,2)=dt*dt/2;T(1,2)=dt;
  auto predicted=f(std::vector<DM>{x,P,dt,y,Q,R,DM::zeros(2,1)});
  EXPECT_LT(double(DM::norm_inf(predicted[0]-DM::mtimes(T,x))),1e-14);
  EXPECT_LT(double(DM::norm_inf(predicted[1]-(DM::mtimes(DM::mtimes(T,P),T.T())+Q))),1e-14);
  auto joint=f(std::vector<DM>{x,P,dt,y,Q,R,DM::ones(2,1)});
  auto synchronous=old(std::vector<DM>{x,P,dt,y,Q,R});
  EXPECT_LT(double(DM::norm_inf(joint[0]-synchronous[0])),1e-14);
  EXPECT_LT(double(DM::norm_inf(joint[1]-synchronous[1])),1e-14);
  for(int field=0;field<2;++field) {
    DM mask=DM::zeros(2,1);mask(field)=1.;
    auto partial=f(std::vector<DM>{x,P,dt,y,Q,R,mask});
    DM changed=y;changed(1-field)=1e6;
    auto ignored=f(std::vector<DM>{x,P,dt,changed,Q,R,mask});
    EXPECT_LT(double(DM::norm_inf(partial[0]-ignored[0])),1e-14);
    EXPECT_GT(double(DM::norm_inf(partial[0]-predicted[0])),1e-5);
  }
}

TEST_F(HardwareObserverTest, WorkerPreservesPosteriorAndInvalidatesStaleDiagnostics) {
  HardwareArmObserver output=h;
  HardwareArmObserverWorker worker(h);
  HardwareArmObserverWorker::Sample sample;
  for(int i=0;i<8;++i) sample.measurement[i]=double(y(i));
  sample.upper.fill(6.);
  now=HardwareArmObserver::Clock::now()-std::chrono::milliseconds(80);
  auto snapshot=[&] {
    for(auto &clock:clocks) for(int j=0;j<3;++j) clock.update(j,now);
    sample.clocks=clocks;
  };
  auto await_sample=[&](double step) {
    const auto deadline=HardwareArmObserver::Clock::now()+std::chrono::milliseconds(100);
    do {
      worker.exchange(sample,output,HardwareArmObserver::Clock::now());
      if(output.sample_step>=step) return true;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    } while(HardwareArmObserver::Clock::now()<deadline);
    return false;
  };
  snapshot(); EXPECT_FALSE(observe()); ASSERT_TRUE(await_sample(1.));
  now+=std::chrono::milliseconds(50);
  sample.input={.03,-.02,.01,.04}; u=DM(std::vector<double>(sample.input.begin(),sample.input.end()));
  snapshot(); ASSERT_TRUE(observe()); ASSERT_TRUE(await_sample(2.));
  EXPECT_EQ(output.step,h.step); EXPECT_EQ(output.valid,1.);
  for(int i=0;i<4;++i) EXPECT_NEAR(output.torque[i],h.torque[i],1e-12);
  for(int i=0;i<16;++i) EXPECT_NEAR(output.covariance[i],h.covariance[i],1e-12);
  for(int i=0;i<1000;++i) worker.exchange(sample,output,HardwareArmObserver::Clock::now());
  EXPECT_EQ(output.step,1.);  // repeated serial packets are not new observations
  worker.exchange(sample,output,HardwareArmObserver::Clock::now()+std::chrono::seconds(1));
  EXPECT_EQ(output.valid,0.);
}

namespace {
// Hold a genuine native update in flight so an invalid packet and its recovery
// deterministically replace the same pending slot before the worker sees either.
class BlockedArmCorrection : public casadi::Callback {
 public:
  explicit BlockedArmCorrection(casadi::Function native) : native_(std::move(native)),
      released_(release_.get_future().share()) { construct("blocked_arm_correction"); }
  casadi_int get_n_in() override { return native_.n_in(); }
  casadi_int get_n_out() override { return native_.n_out(); }
  casadi::Sparsity get_sparsity_in(casadi_int i) override { return native_.sparsity_in(i); }
  casadi::Sparsity get_sparsity_out(casadi_int i) override { return native_.sparsity_out(i); }
  std::vector<DM> eval(const std::vector<DM> &args) const override {
    if(!entered_once_.exchange(true)) {
      entered_.set_value();
      released_.wait();
    }
    return native_(args);
  }
  std::future<void> entered() { return entered_.get_future(); }
  void release() { release_.set_value(); }
 private:
  casadi::Function native_;
  mutable std::atomic<bool> entered_once_{false};
  mutable std::promise<void> entered_;
  std::promise<void> release_;
  std::shared_future<void> released_;
};
}

TEST_F(HardwareObserverTest, WorkerCannotLoseInvalidationWhenPendingPacketIsReplaced) {
  BlockedArmCorrection correction(h.update);
  auto entered=correction.entered();
  h.update=correction;
  HardwareArmObserver output=h;
  HardwareArmObserverWorker worker(h);
  HardwareArmObserverWorker::Sample sample;
  for(int i=0;i<8;++i) sample.measurement[i]=double(y(i));
  sample.upper.fill(6.);
  auto stamp=[&](HardwareArmObserver::Clock::time_point t) {
    for(auto &clock:sample.clocks) for(int j=0;j<3;++j) clock.update(j,t);
  };
  auto poll=[&](double step) {
    const auto deadline=HardwareArmObserver::Clock::now()+std::chrono::milliseconds(100);
    do {
      worker.exchange(sample,output,HardwareArmObserver::Clock::now());
      if(output.sample_step>=step) return true;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    } while(HardwareArmObserver::Clock::now()<deadline);
    return false;
  };
  stamp(HardwareArmObserver::Clock::now()-std::chrono::milliseconds(20));
  // Never leave the blocked callback waiting if the fixture fails early.
  if(!poll(1.)) { correction.release(); FAIL()<<"Worker did not seed"; }
  stamp(HardwareArmObserver::Clock::now());
  worker.exchange(sample,output,HardwareArmObserver::Clock::now());
  const auto waiting=entered.wait_for(std::chrono::milliseconds(100));
  if(waiting!=std::future_status::ready) { correction.release(); FAIL()<<"Correction did not start"; }
  auto bad=sample;
  bad.clocks[0].seen[0]=false;
  worker.exchange(bad,output,HardwareArmObserver::Clock::now());
  EXPECT_EQ(output.valid,0.);
  worker.exchange(sample,output,HardwareArmObserver::Clock::now());
  correction.release();
  ASSERT_TRUE(poll(2.));
  EXPECT_EQ(output.valid,0.); // pre-fault result and duplicate recovery are invalid
  stamp(HardwareArmObserver::Clock::now());
  ASSERT_TRUE(poll(3.));
  EXPECT_EQ(output.valid,0.); // first new complete bundle reseeds
  stamp(HardwareArmObserver::Clock::now());
  ASSERT_TRUE(poll(4.));
  EXPECT_EQ(output.valid,1.);
}

// Reproduces the concurrent DM/Sparsity cache use seen in the hardware core dump.
// The control side must remain free to create matrices while the observer runs.
TEST_F(HardwareObserverTest, WorkerRunsAlongsideControlMatrixOperations) {
  HardwareArmObserver output=h;
  HardwareArmObserverWorker worker(h);
  HardwareArmObserverWorker::Sample sample;
  for(int i=0;i<8;++i) sample.measurement[i]=double(y(i));
  sample.upper.fill(6.);
  const auto deadline=HardwareArmObserver::Clock::now()+std::chrono::seconds(2);
  unsigned iterations=0;
  while(HardwareArmObserver::Clock::now()<deadline && (iterations<10000 || output.step<2)) {
    auto stamp=HardwareArmObserver::Clock::now();
    for(auto &clock:sample.clocks) for(int j=0;j<3;++j) clock.update(j,stamp);
    worker.exchange(sample,output,stamp);
    const int size=3+iterations%10;
    auto matrix=DM::eye(size);
    auto result=DM::mtimes(matrix,matrix)+0.001;
    ASSERT_EQ(result.size1(),size);
    ++iterations;
  }
  EXPECT_GE(iterations,10000u);
  EXPECT_GE(output.step,2.);
}
