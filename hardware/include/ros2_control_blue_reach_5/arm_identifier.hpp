#pragma once
#include "ros2_control_blue_reach_5/arm_identification_sample.hpp"
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace uvms::arm_identification {
using ArmMatrix = Eigen::Matrix<double, 4, 48>;
ArmMatrix regressor(const Sample &, const Eigen::Vector4d &acceleration);
struct Row {
  Eigen::RowVectorXd design;
  double torque = 0, start = 0, end = 0;
  int axis = 0;
};
struct Data {
  Eigen::MatrixXd design;
  Eigen::VectorXd torque;
  std::vector<Row> rows;
};
Data stack(const std::vector<Row> &);
class Window {
public:
  explicit Window(double support = 2., double max_gap = .15,
                  double min_sliding_speed = .005);
  std::vector<Row> observe(const Sample &);
  void reset();

private:
  struct Point {
    Sample sample;
    ArmMatrix momentum, remainder;
  };
  double support_, gap_, min_sliding_speed_, last_row_ = -1;
  std::deque<Point> points_;
};
struct Config {
  bool gripper = false;
  int horizon = 1200;
  double residual_scale = .03, min_information = 1e-5, validation_seconds = 5.,
         budget = 1.;
};
struct Result {
  std::string status = "collecting", detail;
  Eigen::VectorXd candidate, retained;
  double validation_error = -1, retained_error = -1, min_information = 0;
  int rank = 0, dimensions = 0, rows = 0;
  std::array<int, 5> rows_by_axis{};
  double accepted_time = -1;
  bool valid = false;
};
class Identifier {
public:
  explicit Identifier(Config config = {});
  ~Identifier();
  Eigen::VectorXd fit(const Data &);
  void observe(const std::vector<Row> &);
  void reset();
  const Result &result() const { return result_; }
  // Fixed observable subspace at configured torque resolution; no parameter
  // prior.
  void configure(const Sample &);

private:
  struct Solver;
  std::unique_ptr<Solver> solver_;
  Config config_;
  Eigen::MatrixXd basis_;
  Eigen::VectorXd scales_;
  std::vector<Row> training_, validation_, monitoring_;
  Result result_;
  double pending_end_ = -1, monitor_end_ = -1, next_fit_ = 0;
  int contradictions_ = 0;
  std::pair<int, double> information(const Data &) const;
  double error(const Data &, const Eigen::VectorXd &) const;
};
} // namespace uvms::arm_identification
