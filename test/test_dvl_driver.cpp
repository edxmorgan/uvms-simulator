// Loopback only: no sensor or actuator connections.
#include <gtest/gtest.h>
#include "ros2_control_blue_reach_5/dvldriver.hpp"
#include "ros2_control_blue_reach_5/vehicle.hpp"
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>

using namespace std::chrono_literals;
using a50dvl::driver::DVLDriver;
using boost::asio::ip::tcp;

namespace {
struct LoopbackServer {
  boost::asio::io_context io;
  tcp::acceptor listener{io, {tcp::v4(), 0}};
  int port() const { return listener.local_endpoint().port(); }
  void send(const std::vector<std::string> &pieces) {
    tcp::socket peer(io);
    listener.non_blocking(true);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    for (;;) {
      boost::system::error_code error;
      listener.accept(peer, error);
      if (!error) break;
      if ((error != boost::asio::error::would_block && error != boost::asio::error::try_again) ||
          std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("loopback accept failed/timed out");
      std::this_thread::sleep_for(5ms);
    }
    for (const auto &piece : pieces) {
      boost::system::error_code error;
      boost::asio::write(peer, boost::asio::buffer(piece), error);
      if (error) break;
    }
  }
};
nlohmann::json sample() {
  return {{"type","velocity"},{"altitude",1.0},{"covariance",{{1.,0.,0.},{0.,1.,0.},{0.,0.,1.}}},
    {"fom",0.1},{"format","json"},{"status",0},{"time",0.1},
    {"time_of_transmission",100},{"time_of_validity",90},{"transducers",nlohmann::json::array()},
    {"velocity_valid",true},{"vx",0.1},{"vy",0.2},{"vz",0.3}};
}
}

TEST(DVLDriver, FramingAndMalformedMessagesDoNotStopReception) {
  LoopbackServer server;
  auto sender = std::async(std::launch::async, [&] {
    server.send({"{\"vx\":", "1}\n{\"vx\":2}\n{bad json}\n",
                 "{\"type\":\"velocity\"}\n{\"vx\":3}\n"});
  });
  DVLDriver driver;
  std::mutex mutex;
  std::condition_variable received;
  std::vector<int> values;
  driver.subscribe([&](const nlohmann::json &message) {
    const int value = message.at("vx").get<int>(); // Required-field error from consumer.
    std::lock_guard<std::mutex> lock(mutex);
    values.push_back(value);
    received.notify_all();
  });
  driver.start("127.0.0.1", server.port());
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(received.wait_for(lock, 2s, [&] { return values.size() == 3; }));
    EXPECT_EQ(values, (std::vector<int>{1,2,3}));
  }
  driver.stop();
  sender.get();
}

TEST(DVLDriver, OversizedFrameAndEofAllowExplicitRestart) {
  LoopbackServer server;
  auto sender = std::async(std::launch::async, [&] {
    server.send({std::string(DVLDriver::max_message_bytes + 1, 'x')});
    server.send({"{}\n"});
  });
  DVLDriver driver;
  std::promise<void> delivered;
  auto delivery = delivered.get_future();
  driver.subscribe([&](const nlohmann::json &) { delivered.set_value(); });
  driver.start("127.0.0.1", server.port());
  // start must reap a worker that ended on protocol/transport error. No stop first.
  bool restarted = false;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline && !restarted) {
    std::this_thread::sleep_for(5ms);
    try { driver.start("127.0.0.1", server.port()); restarted = true; }
    catch (const std::runtime_error &) {}
  }
  EXPECT_TRUE(restarted);
  EXPECT_EQ(delivery.wait_for(2s), std::future_status::ready);
  driver.stop();
  driver.stop(); // idempotent even after EOF
  sender.get();
}

TEST(DVLDriver, StopUnblocksIdleReadAndCallbackCanBeReplaced) {
  LoopbackServer server;
  std::promise<void> accepted;
  auto sender = std::async(std::launch::async, [&] {
    tcp::socket peer(server.io);
    server.listener.accept(peer);
    accepted.set_value();
    char byte;
    boost::system::error_code error;
    peer.read_some(boost::asio::buffer(&byte, 1), error);
  });
  DVLDriver driver;
  driver.start("127.0.0.1", server.port());
  accepted.get_future().wait();
  for (int i = 0; i < 100; ++i) driver.subscribe([](const nlohmann::json &) {});
  const auto begin = std::chrono::steady_clock::now();
  driver.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s);
  sender.get();
}

TEST(DVLMessage, RejectsInvalidTimestampsAndCovarianceBeforeAcceptance) {
  using blue::dynamics::DVLVelocityMessage;
  auto good = sample();
  EXPECT_NO_THROW(DVLVelocityMessage::from_json(good));
  for (const auto &stamp : {nlohmann::json(1.5), nlohmann::json(-1), nlohmann::json(true),
                           nlohmann::json(std::numeric_limits<unsigned long long>::max())}) {
    auto invalid = good;
    invalid["time_of_validity"] = stamp;
    EXPECT_THROW(DVLVelocityMessage::from_json(invalid), std::invalid_argument);
  }
  auto invalid = good;
  invalid["time_of_validity"] = 10000000;
  invalid["covariance"] = {{-1.,0.,0.},{0.,1.,0.},{0.,0.,1.}};
  EXPECT_THROW(DVLVelocityMessage::from_json(invalid), std::invalid_argument);
  invalid["velocity_valid"] = false;
  invalid["covariance"] = {{0.,0.,0.},{0.,0.,0.},{0.,0.,0.}};
  EXPECT_NO_THROW(DVLVelocityMessage::from_json(invalid));
}
