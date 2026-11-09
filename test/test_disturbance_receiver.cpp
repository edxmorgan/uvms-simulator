#include <gtest/gtest.h>
#include <limits>
#include "ros2_control_blue_reach_5/disturbance_receiver.hpp"
using namespace ros2_control_blue_reach_5;

TEST(DisturbanceLease, OwnershipExpiryAndRelease) {
  DisturbanceLease lease("robot_1_");
  auto now=DisturbanceLease::Clock::now();
  msg::DisturbanceCommand a;
  a.header.frame_id="robot_1_base_link"; a.joint_names=lease.names();
  a.session_id="A"; a.sequence=1; a.acquire=true;
  EXPECT_TRUE(lease.receive(a,now));
  auto b=a; b.session_id="B";
  EXPECT_FALSE(lease.receive(b,now));
  a.acquire=false; a.sequence=2; a.body_wrench.force.x=3.; a.joint_torques[0]=.1;
  EXPECT_TRUE(lease.receive(a,now));
  EXPECT_FALSE(lease.receive(a,now)); // duplicate cannot renew the lease
  EXPECT_DOUBLE_EQ(lease.sample(now).body_wrench.force.x,3.);
  EXPECT_DOUBLE_EQ(lease.sample(now).joint_torques[0],.1);
  EXPECT_DOUBLE_EQ(lease.sample(now+std::chrono::seconds(1)).body_wrench.force.x,0.);
  EXPECT_FALSE(lease.receive(a,now+std::chrono::seconds(2))); // expired session cannot resume
  EXPECT_TRUE(lease.receive(b,now+std::chrono::seconds(2)));
  b.release=true; b.acquire=false; b.sequence=2;
  EXPECT_TRUE(lease.receive(b,now+std::chrono::seconds(2)));
  b.acquire=true; b.release=false; b.sequence=3;
  EXPECT_FALSE(lease.receive(b,now+std::chrono::seconds(2)));
  a.acquire=true; a.body_wrench.force.x=0.; a.joint_torques.fill(0.); a.sequence=10;
  EXPECT_FALSE(lease.receive(a,now+std::chrono::seconds(2))); // older retired owner also stays retired
}
TEST(DisturbanceLease, ValidationAndReset) {
  DisturbanceLease lease("robot_1_");
  auto now=DisturbanceLease::Clock::now();
  msg::DisturbanceCommand a;
  a.header.frame_id="robot_1_base_link"; a.joint_names=lease.names();
  a.session_id="A"; a.sequence=1; a.acquire=true;
  a.body_wrench.force.x=1.; EXPECT_FALSE(lease.receive(a,now));
  a.body_wrench.force.x=0.; a.header.frame_id="world"; EXPECT_FALSE(lease.receive(a,now));
  a.header.frame_id="robot_1_base_link"; a.joint_names[0]="wrong"; EXPECT_FALSE(lease.receive(a,now));
  a.joint_names=lease.names(); EXPECT_TRUE(lease.receive(a,now));
  a.acquire=false; a.sequence=2; a.joint_torques[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(lease.receive(a,now));
  lease.clear(); EXPECT_FALSE(lease.active(now));
  EXPECT_DOUBLE_EQ(lease.sample(now).joint_torques[0],0.);
}
