#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/hardware_arm_identification.hpp"
using uvms::arm_identification::HardwareMeasurements;
const std::array<double,4> lower{-10,-10,-10,-10}, upper{10,10,10,10};
void frame(HardwareMeasurements &m,double t) {
  for(int j=0;j<4;++j) {
    m.update(j,0,t,2*t); m.update(j,1,t+.005,2);
    m.update(j,2,t+.01,3*(t+.01));
  }
}
TEST(HardwareArmIdentification, InterpolatesWithoutExtrapolatingAndIntegratesMeasuredTorque) {
  HardwareMeasurements m; frame(m,0);frame(m,.05);
  ASSERT_TRUE(m.snapshot(.061,lower,upper)); double start=m.sample.time;
  for(int k=2;k<=20;++k) {frame(m,.05*k);ASSERT_TRUE(m.snapshot(.05*k+.011,lower,upper));}
  EXPECT_NEAR(m.sample.time,1,1e-12);
  EXPECT_NEAR(m.sample.position[0],2,1e-12);
  EXPECT_NEAR(m.sample.impulse[0],1.5*(1-start*start),1e-12);
  EXPECT_FALSE(m.snapshot(1.011,lower,upper));
  EXPECT_EQ(m.sample.valid[4],0);
}
TEST(HardwareArmIdentification, MissingChannelAndStaleFeedbackCannotAccumulateRows) {
  HardwareMeasurements m;frame(m,0);frame(m,.05);ASSERT_TRUE(m.snapshot(.061,lower,upper));
  for(int k=2;k<4;++k) for(int j=0;j<4;++j) {m.update(j,0,.05*k,1);m.update(j,1,.05*k,2);}
  m.snapshot(.161,lower,upper);EXPECT_NEAR(m.sample.time,.06,1e-12);
  EXPECT_FALSE(m.snapshot(.3,lower,upper));EXPECT_EQ(m.sample.supported,0);
  frame(m,.35);frame(m,.4);ASSERT_TRUE(m.snapshot(.411,lower,upper));
  EXPECT_EQ(m.sample.impulse[0],0);
}
TEST(HardwareArmIdentification, InvalidSamplesAndContactsInvalidateAllArmEquations) {
  HardwareMeasurements m;frame(m,0);frame(m,.05);ASSERT_TRUE(m.snapshot(.061,lower,upper));
  auto generation=m.sample.generation;
  m.update(1,2,.07,std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(m.sample.supported,0);EXPECT_GT(m.sample.generation[0],generation[0]);
  frame(m,.1);frame(m,.15);auto limit=upper;limit[0]=.3;
  ASSERT_TRUE(m.snapshot(.161,lower,limit));
  for(int j=0;j<4;++j) EXPECT_EQ(m.sample.valid[j],0);
}
