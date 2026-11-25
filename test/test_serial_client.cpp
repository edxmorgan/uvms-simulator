#include <gtest/gtest.h>
#include <pty.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "ros2_control_blue_reach_5/serial_client.hpp"
#include "ros2_control_blue_reach_5/driver.hpp"

using namespace alpha::driver;

TEST(Driver, ConcurrentCommandsAndPollingPreserveEveryDeviceReply)
{
  int master=-1, slave=-1; char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  std::atomic<bool> running{true};
  std::atomic<unsigned> malformed{0}, commands{0};
  std::array<std::atomic<unsigned>,5> replies{};
  // A full-duplex peer decodes the actual outgoing frames, then answers
  // requests. Unlike injected RX-only traffic, this exercises shared TX.
  std::thread peer([&] {
    std::vector<unsigned char> frame;
    while(running.load()) {
      pollfd fd{master,POLLIN,0};
      if(::poll(&fd,1,20)<=0) continue;
      unsigned char byte;
      if(read(master,&byte,1)!=1) continue;
      frame.push_back(byte);
      if(byte!=0) continue;
      try {
        const auto packet=Packet::decode(frame);
        if(packet.getPacketId()==PacketId::PacketID_REQUEST_PACKET) {
          for(auto field:packet.getData()) {
            const auto response=Packet(static_cast<PacketId>(field),
              packet.getDeviceId(),{0,0,0,0}).encode();
            if(write(master,response.data(),response.size())!=
                static_cast<ssize_t>(response.size())) ++malformed;
          }
        } else if(packet.getPacketId()==PacketId::PacketID_CURRENT) ++commands;
      } catch(const std::exception &) { ++malformed; }
      frame.clear();
    }
  });
  Driver driver;
  for(auto field:{PacketId::PacketID_POSITION,PacketId::PacketID_VELOCITY,
                 PacketId::PacketID_CURRENT})
    driver.subscribe(field,[&](Packet packet) {
      const auto id=static_cast<unsigned>(packet.getDeviceId());
      if(id>=1 && id<=5) ++replies[id-1];
    });
  driver.start(name,1,false);
  std::thread polling([&] {
    for(unsigned tick=0;tick<40;++tick) {
      for(auto field:{PacketId::PacketID_VELOCITY,PacketId::PacketID_POSITION,
                     PacketId::PacketID_CURRENT})
        for(unsigned id=1;id<=5;++id) driver.request(field,static_cast<DeviceId>(id));
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
  for(unsigned tick=0;tick<100;++tick) {
    for(unsigned id=1;id<=5;++id) {
      driver.setCurrent(0.f,static_cast<DeviceId>(id));
      driver.setVelocity(0.f,static_cast<DeviceId>(id));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  polling.join();
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(std::chrono::steady_clock::now()<deadline) {
    bool complete=commands.load()==500;
    for(auto &count:replies) complete=complete && count.load()==120;
    if(complete) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  driver.stop();
  running=false; peer.join();
  EXPECT_EQ(malformed.load(),0u);
  EXPECT_EQ(commands.load(),500u);
  for(auto &count:replies) EXPECT_EQ(count.load(),120u);
  EXPECT_FALSE(driver.read_failed());
  EXPECT_FALSE(driver.write_failed());
  close(master);close(slave);
}

TEST(SerialClient, WriteFailureIsLatchedUntilReconnect)
{
  SerialClient client;
  const Packet packet(PacketId::PacketID_POSITION, static_cast<DeviceId>(3), {0,0,0,0});
  EXPECT_FALSE(client.write_failed());
  EXPECT_FALSE(client.send(packet));
  EXPECT_TRUE(client.write_failed());

  int master=-1, slave=-1;
  char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  client.connect(name,100);
  EXPECT_FALSE(client.write_failed());
  EXPECT_TRUE(client.send(packet));
  EXPECT_FALSE(client.write_failed());
  // A vanished peer must surface as a latched fault without throwing into
  // request/heartbeat threads. Disconnect must not erase that evidence.
  close(master);
  EXPECT_FALSE(client.send(packet));
  EXPECT_TRUE(client.write_failed());
  client.disconnect();
  EXPECT_TRUE(client.write_failed());
  close(slave);
}

TEST(SerialClient, ReceivesWhileCallbacksAreRegisteredAndAllowsReentrantRegistration)
{
  int master=-1, slave=-1;
  char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  SerialClient client;
  client.connect(name,100);
  std::atomic<unsigned> received{0};
  client.registerCallback(PacketId::PacketID_POSITION,[&](Packet) {
    if (received.fetch_add(1)==0)
      client.registerCallback(PacketId::PacketID_CURRENT,[](Packet) {});
  });
  const unsigned char delimiter=0;
  ASSERT_EQ(write(master,&delimiter,1),1);
  const Packet packet(PacketId::PacketID_POSITION,static_cast<DeviceId>(3),{0,0,0,0});
  const auto bytes=packet.encode();
  std::thread registrar([&] {
    for(unsigned i=0;i<1000;++i)
      client.registerCallback(PacketId::PacketID_VELOCITY,[](Packet) {});
  });
  for(unsigned i=0;i<50;++i) {
    EXPECT_EQ(write(master,bytes.data(),bytes.size()),static_cast<ssize_t>(bytes.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  registrar.join();
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(received.load()<50 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(received.load(),50u);
  const auto report=client.diagnostics();
  EXPECT_NE(report.find("decoded_count=50"),std::string::npos);
  // Stop RX before destroying callbacks' captured state.
  client.disconnect();
  close(master);close(slave);
}

TEST(SerialClient, FirstPacketReconnectAndIdempotentDisconnect)
{
  int master=-1, slave=-1; char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  SerialClient client;
  std::atomic<unsigned> received{0};
  client.registerCallback(PacketId::PacketID_POSITION,[&](Packet){++received;});
  const auto bytes=Packet(PacketId::PacketID_POSITION,static_cast<DeviceId>(3),{0,0,0,0}).encode();
  for (unsigned cycle=1;cycle<=2;++cycle) {
    client.connect(name,100);
    EXPECT_THROW(client.connect(name,100),std::logic_error);
    EXPECT_EQ(write(master,bytes.data(),bytes.size()),static_cast<ssize_t>(bytes.size()));
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while(received.load()<cycle && std::chrono::steady_clock::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(received.load(),cycle);
    client.disconnect();
    EXPECT_NO_THROW(client.disconnect());
    EXPECT_FALSE(client.active());
  }
  close(master);close(slave);
}

TEST(SerialClient, FailedConnectAndDestructorCleanUp)
{
  SerialClient client;
  EXPECT_THROW(client.connect("/dev/null",100),std::runtime_error);
  EXPECT_NO_THROW(client.disconnect());
  int master=-1, slave=-1;char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  { SerialClient scoped; scoped.connect(name,100); }
  client.connect(name,100);
  close(master);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(client.active() && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_FALSE(client.active());
  EXPECT_TRUE(client.read_failed());
  client.disconnect();close(slave);
}

TEST(SerialClient, ClosedHandleCannotWriteToReusedDescriptor)
{
  int master=-1, slave=-1;char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  SerialClient client;client.connect(name,100);client.disconnect();
  // tmpfile typically reuses the just-closed descriptor. No data may escape
  // even if the operating system gives that descriptor to another owner.
  FILE *sink=tmpfile();ASSERT_NE(sink,nullptr);
  const Packet packet(PacketId::PacketID_POSITION,static_cast<DeviceId>(3),{0,0,0,0});
  EXPECT_FALSE(client.send(packet));
  EXPECT_EQ(lseek(fileno(sink),0,SEEK_END),0);
  fclose(sink);close(master);close(slave);
}

TEST(Driver, StopIsIdempotentAndDestructorJoins)
{
  Driver driver;
  EXPECT_NO_THROW(driver.stop());
  EXPECT_THROW(driver.start("/dev/null",1,false),std::runtime_error);
  EXPECT_NO_THROW(driver.stop());
  int master=-1, slave=-1;char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  for (int cycle=0;cycle<2;++cycle) {
    driver.start(name,1,false);
    EXPECT_THROW(driver.start(name,1,false),std::logic_error);
    driver.stop();
    EXPECT_NO_THROW(driver.stop());
  }
  { Driver scoped;scoped.start(name,1,false); }
  close(master);close(slave);
}

TEST(SerialClient, MalformedAndOversizedFramesRecoverAtDelimiter)
{
  int master=-1, slave=-1;char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  std::atomic<unsigned> received{0};
  SerialClient client;
  client.registerCallback(PacketId::PacketID_POSITION,[&](Packet){++received;});
  client.connect(name,100);
  const auto valid=Packet(PacketId::PacketID_POSITION,static_cast<DeviceId>(3),{1,2,3,4}).encode();
  auto corrupt=valid;corrupt[1]^=0x40;
  EXPECT_EQ(write(master,corrupt.data(),corrupt.size()),static_cast<ssize_t>(corrupt.size()));
  std::vector<unsigned char> oversized(300,1);oversized.push_back(0);
  EXPECT_EQ(write(master,oversized.data(),oversized.size()),static_cast<ssize_t>(oversized.size()));
  EXPECT_EQ(write(master,valid.data(),valid.size()),static_cast<ssize_t>(valid.size()));
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(received.load()<1 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  client.disconnect();
  EXPECT_EQ(received.load(),1u);
  EXPECT_NE(client.diagnostics().find("rejected_frames_or_callbacks=2"),std::string::npos);
  close(master);close(slave);
}

TEST(SerialClient, IndependentDevicesRemainReceivableAfterSelectiveOutage)
{
  int master=-1, slave=-1;
  char name[256]{};
  ASSERT_EQ(openpty(&master,&slave,name,nullptr,nullptr),0);
  std::array<std::atomic<unsigned>,5> received{};
  SerialClient client;
  client.registerCallback(PacketId::PacketID_POSITION,[&](Packet packet) {
    const auto device=static_cast<unsigned>(packet.getDeviceId());
    if(device>=1 && device<=5) ++received[device-1];
  });
  client.connect(name,100);
  auto send=[&](unsigned device) {
    const auto bytes=Packet(PacketId::PacketID_POSITION,
      static_cast<DeviceId>(device),{0,0,0,0}).encode();
    EXPECT_EQ(write(master,bytes.data(),bytes.size()),static_cast<ssize_t>(bytes.size()));
  };
  auto await_count=[&](unsigned device,unsigned expected) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    while(received[device-1].load()<expected && std::chrono::steady_clock::now()<deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    EXPECT_EQ(received[device-1].load(),expected);
  };
  for(unsigned device=1;device<=5;++device) send(device);
  for(unsigned device=1;device<=5;++device) await_count(device,1);
  // Match the reported pattern: base (5) replies while downstream IDs go silent.
  for(unsigned tick=0;tick<12;++tick) {
    send(5);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  for(unsigned device=1;device<=4;++device) EXPECT_EQ(received[device-1].load(),1u);
  await_count(5,13);
  for(unsigned device=1;device<=5;++device) send(device);
  for(unsigned device=1;device<=5;++device) await_count(device,device==5?14:2);
  EXPECT_TRUE(client.active());
  EXPECT_FALSE(client.read_failed());
  EXPECT_FALSE(client.write_failed());
  EXPECT_NE(client.diagnostics().find("rejected_frames_or_callbacks=0"),std::string::npos);
  client.disconnect();
  close(master);close(slave);
}

TEST(SerialTrace, DistinguishesSelectiveReplyLossFromRequestAndCallbackStalls)
{
  SerialTrace trace;
  const auto packet = PacketId::PacketID_POSITION;
  constexpr std::int64_t now = 2000000000;
  trace.record(1, packet, SerialTrace::Stage::requested, now-10000000);
  trace.record(1, packet, SerialTrace::Stage::decoded, now-600000000);
  trace.record(1, packet, SerialTrace::Stage::dispatched, now-600000000);
  trace.record(2, packet, SerialTrace::Stage::requested, now-600000000);
  trace.record(3, packet, SerialTrace::Stage::decoded, now-10000000);
  trace.record(5, packet, SerialTrace::Stage::decoded, now-10000000);
  trace.record(5, packet, SerialTrace::Stage::dispatched, now-10000000);
  const auto report = trace.report(now);
  EXPECT_NE(report.find("device=1/position observation=requests_written_without_recent_decoded_reply"), std::string::npos);
  EXPECT_NE(report.find("device=2/position observation=no_recent_request_or_decoded_reply"), std::string::npos);
  EXPECT_NE(report.find("device=3/position observation=decoded_without_recent_callback"), std::string::npos);
  EXPECT_NE(report.find("device=5/position observation=callback_recent"), std::string::npos);
  trace.record(1, packet, SerialTrace::Stage::decoded, now);
  trace.record(1, packet, SerialTrace::Stage::dispatched, now);
  EXPECT_NE(trace.report(now).find("device=1/position observation=callback_recent"), std::string::npos);
}
