#pragma once

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <optional>
#include <functional>
#include <string>
#include <vector>

namespace uvms::identification {
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Parameters = Eigen::Matrix<double, 27, 1>;
using Regressor = Eigen::Matrix<double, 6, 27>;

// Already converted to NED/FRD at the transport boundary. No truth/model fields.
struct Measurement {
  double generation, step, time, period, depth;
  Eigen::Vector3d attitude;
  Vector6 velocity, wrench;
};
struct Window {
  std::size_t epoch;
  double start, end;
  Eigen::MatrixXd design;
  Eigen::VectorXd wrench;
  std::vector<double> times;
  // Earliest measurement used by each row; empty for legacy pointwise fixtures.
  std::vector<double> support_starts;
};
class LiveWindow {
 public:
  explicit LiveWindow(std::size_t horizon = 4200, double max_gap = .05);
  bool push(const Measurement &);
  void reset();
  std::optional<Window> snapshot() const;
  std::size_t epoch() const { return epoch_; }
  std::size_t size() const { return rows_.size(); }
 private:
  struct Row { double start, time; Regressor design; Vector6 wrench; };
  std::size_t horizon_, epoch_ = 0;
  double max_gap_;
  std::optional<Measurement> previous_, older_;
  Regressor previous_bias_, older_bias_;
  std::deque<Row> rows_;
};
struct Fit {
  Parameters parameters;
  Vector6 residual_rms;
  double solver_seconds = 0, wall_seconds = 0, relative_change = 0;
  int rank = 0, iterations = 0;
};
class Identifier {
 public:
  Identifier(double weight_min, double weight_max, double budget = .5);
  ~Identifier();
  // Exercise the same constrained solver without consuming measurements or publishing a fit.
  void preflight();
  Identifier(const Identifier &) = delete;
  Identifier & operator=(const Identifier &) = delete;
  // Worker-thread only. Throws on invalid, late, infeasible or unconverged fits.
  Fit fit(const Eigen::MatrixXd &, const Eigen::VectorXd &, bool robust = true);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
Eigen::Matrix<double, 6, 6> inertia_matrix(const Parameters &);

// Engineering-scaled sensitivity information, NOT a posterior covariance.
// Fixed scaling prevents small observed column norms from manufacturing excitation.
struct RetentionConfig {
  Parameters parameter_scales;
  Vector6 residual_scales;
  double min_information = 1e-4;
  double validation_seconds = 5., validation_gap = .1;
  double max_validation_error = 1., change_error = 3.;
  double improvement_fraction = .05, information_gain = 1.2;
  int change_blocks = 3;
  RetentionConfig();
};
struct Information {
  int rank = 0, dimensions = 0;
  double minimum = 0, condition = 0;
  bool sufficient = false, net_weight_only = false;
};
struct RetentionResult {
  std::optional<Fit> candidate;
  std::string status, detail;
  Information information;
  bool available = false, valid = false, accepted = false;
  bool retained_net_weight_only = false;
  Parameters parameters = Parameters::Zero();
  double accepted_time = 0, information_at_acceptance = 0;
  // -1 means no independent validation block was evaluated on this update.
  double validation_error = -1, retained_error = -1;
  int contradictions = 0;
  bool retained_rms_available = false;
  Vector6 retained_rms = Vector6::Zero();
  std::size_t retained_rms_samples = 0;
  double retained_rms_start = 0, retained_rms_end = 0;
};
class ModelRetention {
 public:
  using Solver = std::function<Fit(const Eigen::MatrixXd &, const Eigen::VectorXd &)>;
  ModelRetention(RetentionConfig config, bool fixed_weight);
  // Single worker only. Epoch changes invalidate all history. No control writes.
  RetentionResult update(const Window &, const Solver &);
  Information information(const Eigen::MatrixXd &) const;
 private:
  struct Saved { Fit fit; Window training; Information info; double accepted_time = 0; };
  RetentionConfig config_;
  bool fixed_weight_, stale_ = false;
  std::optional<std::size_t> epoch_;
  std::optional<Saved> retained_, pending_;
  double last_validation_end_ = 0;
  int contradictions_ = 0;
  double error(const Window &, const Parameters &) const;
};
}  // namespace uvms::identification
