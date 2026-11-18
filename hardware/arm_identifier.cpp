#include "ros2_control_blue_reach_5/arm_identifier.hpp"
#include "fusion.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

namespace uvms::arm_identification {
using namespace mosek::fusion;
using namespace monty;
struct Identifier::Solver {
  bool gripper;
  explicit Solver(bool value) : gripper(value) {}
  Eigen::VectorXd solve(const Eigen::MatrixXd &a, const Eigen::VectorXd &b,
                        double budget) {
    const int n = a.cols();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(a);
    Eigen::MatrixXd r = qr.matrixQR().topRows(n).triangularView<Eigen::Upper>();
    Eigen::VectorXd target = (qr.householderQ().adjoint() * b).head(n);
    Model::t model = new Model("arm_identification");
    try {
      Expression::t theta;
      Variable::t gripper_parameters;
      std::vector<Variable::t> inertias, frictions;
      if (gripper) {
        gripper_parameters = model->variable(3, Domain::greaterThan(0.));
        theta = gripper_parameters;
        model->constraint(theta->index(0), Domain::inRange(.001, 10000.));
      } else {
        // Parameterize physical inertia directly. Separate equality-linked
        // copies can lose physical feasibility in unobservable directions.
        for (int j = 0; j < 4; ++j) {
          auto p = model->variable(Domain::inPSDCone(4));
          auto friction = model->variable(2, Domain::greaterThan(0.));
          inertias.push_back(p);
          frictions.push_back(friction);
          auto diagonal = [&](int k) {
            return Expr::add(p->index(k, k), 1e-7);
          };
          auto mass = diagonal(3);
          model->constraint(mass, Domain::inRange(.001, 10.));
          Expression::t link = mass;
          for (int k = 0; k < 3; ++k)
            link = Expr::vstack(link, p->index(k, 3));
          link = Expr::vstack(link, Expr::add(diagonal(1), diagonal(2)));
          link = Expr::vstack(link, Expr::add(diagonal(0), diagonal(2)));
          link = Expr::vstack(link, Expr::add(diagonal(0), diagonal(1)));
          for (auto pair : {std::pair<int, int>{0, 1}, {0, 2}, {1, 2}})
            link = Expr::vstack(link,
                                Expr::neg(p->index(pair.first, pair.second)));
          link = Expr::vstack(link, friction);
          theta = j == 0 ? link : Expr::vstack(theta, link);
        }
      }
      // Fusion's affine conic interface uses scalar variables here; link the
      // physical parameterization without independently scaled columns.
      auto coefficients = model->variable(n, Domain::unbounded());
      model->constraint(Expr::sub(coefficients, theta), Domain::equalsTo(0.));
      std::vector<double> rm(n * n);
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
          rm[i * n + j] = r(i, j);
      auto t = model->variable(1, Domain::unbounded());
      auto residual =
          Expr::sub(Expr::mul(Matrix::dense(n, n, new_array_ptr<double>(rm)),
                              coefficients),
                    new_array_ptr<double>(
                        std::vector<double>(target.data(), target.data() + n)));
      model->constraint(Expr::vstack(t, Expr::vstack(.5, residual)),
                        Domain::inRotatedQCone());
      model->objective(ObjectiveSense::Minimize, t);
      model->setSolverParam("numThreads", 1);
      model->setSolverParam("optimizerMaxTime", budget);
      model->setSolverParam("intpntCoTolRelGap", 1e-9);
      model->setSolverParam("intpntCoTolPfeas", 1e-9);
      model->setSolverParam("intpntCoTolDfeas", 1e-9);
      model->solve();
      if (model->getPrimalSolutionStatus() != SolutionStatus::Optimal)
        throw std::runtime_error(
            "Arm solver did not converge (status=" +
            std::to_string(int(model->getPrimalSolutionStatus())) + ")");
      Eigen::VectorXd p(n);
      if (gripper) {
        auto values = gripper_parameters->level();
        for (int i = 0; i < n; ++i)
          p[i] = (*values)[i];
      } else {
        for (int j = 0; j < 4; ++j) {
          auto values = inertias[j]->level();
          Eigen::Matrix4d physical;
          for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 4; ++col)
              physical(row, col) = (*values)[4 * row + col];
          physical.diagonal().array() += 1e-7;
          auto v = p.segment<12>(12 * j);
          v[0] = physical(3, 3);
          v.segment<3>(1) = physical.topRightCorner<3, 1>();
          const Eigen::Matrix3d inertia =
              physical.topLeftCorner<3, 3>().trace() *
                  Eigen::Matrix3d::Identity() -
              physical.topLeftCorner<3, 3>();
          v.segment<6>(4) << inertia(0, 0), inertia(1, 1), inertia(2, 2),
              inertia(0, 1), inertia(0, 2), inertia(1, 2);
          auto f = frictions[j]->level();
          v[10] = (*f)[0];
          v[11] = (*f)[1];
        }
      }
      if (!p.allFinite())
        throw std::runtime_error("Nonfinite arm candidate");
      if (gripper) {
        if (p[0] < .001 - 1e-7 || p[0] > 10000 + 1e-7 ||
            p.tail(2).minCoeff() < -1e-7)
          throw std::runtime_error("Nonphysical gripper candidate");
      } else
        for (int j = 0; j < 4; ++j) {
          auto t = p.segment<12>(12 * j);
          Eigen::Matrix3d inertia;
          inertia << t[4], t[7], t[8], t[7], t[5], t[9], t[8], t[9], t[6];
          Eigen::Matrix4d pseudo = Eigen::Matrix4d::Zero();
          pseudo.topLeftCorner<3, 3>() =
              .5 * inertia.trace() * Eigen::Matrix3d::Identity() - inertia;
          pseudo.topRightCorner<3, 1>() = t.segment<3>(1);
          pseudo.bottomLeftCorner<1, 3>() = t.segment<3>(1).transpose();
          pseudo(3, 3) = t[0];
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(pseudo);
          if (t[0] < .001 - 1e-7 || t[0] > 10 + 1e-7 ||
              t.tail<2>().minCoeff() < -1e-7 || eig.info() != Eigen::Success ||
              eig.eigenvalues().minCoeff() <= 1e-9) {
            std::ostringstream reason;
            reason << "Nonphysical arm candidate: link=" << j
                   << " mass=" << t[0]
                   << " minimum friction=" << t.tail<2>().minCoeff()
                   << " minimum pseudo-inertia eigenvalue="
                   << eig.eigenvalues().minCoeff();
            throw std::runtime_error(reason.str());
          }
        }
      model->dispose();
      return p;
    } catch (...) {
      model->dispose();
      throw;
    }
  }
};
Identifier::Identifier(Config c) : solver_(new Solver(c.gripper)), config_(c) {
  if (c.horizon < 60 || c.horizon > 100000 ||
      !std::isfinite(c.residual_scale) || c.residual_scale <= 0 ||
      !std::isfinite(c.min_information) || c.min_information <= 0 ||
      !std::isfinite(c.validation_seconds) || c.validation_seconds <= 0 ||
      !std::isfinite(c.budget) || c.budget <= 0)
    throw std::invalid_argument("Invalid arm identification configuration");
  configure(Sample{});
}
Identifier::~Identifier() = default;
void Identifier::reset() {
  training_.clear();
  validation_.clear();
  monitoring_.clear();
  result_ = Result{};
  result_.dimensions = basis_.cols();
  pending_end_ = monitor_end_ = -1;
  next_fit_ = 0;
  contradictions_ = 0;
}
void Identifier::configure(const Sample &s) {
  if (config_.gripper) {
    scales_ = Eigen::Vector3d(300, 400, 100);
    basis_ = Eigen::Matrix3d::Identity();
  } else {
    scales_.resize(48);
    for (int j = 0; j < 4; ++j)
      scales_.segment<12>(12 * j) << .5, .05, .05, .05, .2, .2, .2, .05, .05,
          .05, 2., .2;
    // Structural rank is determined once from the model, not from noisy live
    // data.
    std::mt19937 rng(729);
    std::uniform_real_distribution<double> u(-1, 1);
    Eigen::MatrixXd a(400, 48);
    for (int i = 0; i < 100; ++i) {
      Sample p = s;
      Eigen::Vector4d acc;
      for (int j = 0; j < 4; ++j) {
        p.position[j] = u(rng);
        p.velocity[j] = u(rng);
        acc[j] = u(rng);
      }
      a.middleRows<4>(4 * i) = regressor(p, acc) * scales_.asDiagonal();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinV);
    // Discard directions below the configured measurement resolution even under
    // this fixed rich-motion reference. Rounded mounting angles otherwise
    // create spurious ~1e-6 gravity directions that no experiment can resolve
    // here.
    const double resolution =
        config_.residual_scale * std::sqrt(a.rows() * config_.min_information);
    int rank = (svd.singularValues().array() > resolution).count();
    basis_ = svd.matrixV().leftCols(rank);
  }
  reset();
}
std::pair<int, double> Identifier::information(const Data &d) const {
  if (d.rows.empty())
    return {0, 0};
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      d.design * scales_.asDiagonal() * basis_ /
      (config_.residual_scale * std::sqrt(double(d.rows.size()))));
  auto values = svd.singularValues();
  int rank = (values.array().square() >= config_.min_information).count();
  return {rank, values.size() == basis_.cols()
                    ? values.tail(1)[0] * values.tail(1)[0]
                    : 0.};
}
Eigen::VectorXd Identifier::fit(const Data &d) {
  const int n = config_.gripper ? 3 : 48;
  if (d.design.cols() != n || d.design.rows() < n ||
      d.torque.size() != d.design.rows() || !d.design.allFinite() ||
      !d.torque.allFinite())
    throw std::invalid_argument("Invalid arm fit data");
  auto start = std::chrono::steady_clock::now();
  auto remaining = [&]() {
    double t = config_.budget - std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
    if (t <= 0)
      throw std::runtime_error("Arm fit budget exceeded");
    return t;
  };
  Eigen::MatrixXd a = d.design / config_.residual_scale;
  Eigen::VectorXd b = d.torque / config_.residual_scale;
  const double magnitude = std::max(1., b.norm() / std::sqrt(double(b.size())));
  a /= magnitude;
  b /= magnitude;
  auto p = solver_->solve(a, b, remaining());
  // Huber loss in torque-noise units; no nominal parameter penalty.
  bool converged = false;
  for (int k = 0; k < 16; ++k) {
    Eigen::VectorXd weights =
        (1.5 / magnitude / (a * p - b).array().abs().max(1e-12)).min(1.).sqrt();
    auto updated = solver_->solve(weights.asDiagonal() * a,
                                  weights.asDiagonal() * b, remaining());
    double change = (a * (updated - p)).norm() / std::max(1., (a * p).norm());
    p = updated;
    if (change < 1e-5) {
      converged = true;
      break;
    }
  }
  remaining();
  if (!converged)
    throw std::runtime_error("Arm Huber fit did not converge");
  return p;
}
double Identifier::error(const Data &d, const Eigen::VectorXd &p) const {
  Eigen::VectorXd residual = d.design * p - d.torque;
  double sums[5] = {}, counts[5] = {};
  for (size_t i = 0; i < d.rows.size(); ++i) {
    int j = d.rows[i].axis;
    sums[j] += residual[i] * residual[i];
    counts[j]++;
  }
  double worst = 0;
  for (int j = config_.gripper ? 4 : 0; j < (config_.gripper ? 5 : 4); ++j) {
    if (counts[j] < 10)
      return -1;
    worst = std::max(worst,
                     std::sqrt(sums[j] / counts[j]) / config_.residual_scale);
  }
  return worst;
}
void Identifier::observe(const std::vector<Row> &rows) {
  bool added = false;
  for (const auto &r : rows)
    if ((r.axis == 4) == config_.gripper) {
      added = true;
      training_.push_back(r);
      if (pending_end_ >= 0 && r.start > pending_end_ + .1)
        validation_.push_back(r);
      if (result_.valid && r.start > monitor_end_ + .1)
        monitoring_.push_back(r);
    }
  if (!added)
    return;
  if (training_.size() > size_t(config_.horizon))
    training_.erase(training_.begin(), training_.end() - config_.horizon);
  result_.rows = training_.size();
  result_.rows_by_axis.fill(0);
  for (const auto &row : training_)
    if (row.axis >= 0 && row.axis < 5) ++result_.rows_by_axis[row.axis];
  if (training_.empty())
    return;
  const double now = training_.back().end;
  auto ready = [&](const std::vector<Row> &v) {
    return !v.empty() &&
           v.back().end - v.front().end >= config_.validation_seconds;
  };
  if (result_.valid && ready(monitoring_)) {
    double e = error(stack(monitoring_), result_.retained);
    result_.retained_error = e;
    if (e >= 0)
      contradictions_ = e > 3 ? contradictions_ + 1 : 0;
    monitoring_.clear();
    monitor_end_ = now;
    if (contradictions_ >= 3) {
      result_.valid = false;
      result_.detail = "Retained model invalidated by later measurements";
      validation_.clear();
      pending_end_ = -1;
    }
  }
  if (pending_end_ >= 0 && ready(validation_)) {
    const auto d = stack(validation_);
    double e = error(d, result_.candidate);
    result_.validation_error = e;
    if (e >= 0 && information(d).first == basis_.cols()) {
      if (e <= 1) {
        result_.retained = result_.candidate;
        result_.valid = true;
        result_.accepted_time = now;
        result_.status = "validated";
        result_.detail.clear();
        monitor_end_ = now;
        monitoring_.clear();
        contradictions_ = 0;
      } else {
        std::ostringstream text;
        text << "Candidate rejected: independent torque RMS / limit = " << e;
        result_.detail = text.str();
      }
      validation_.clear();
      pending_end_ = -1;
      next_fit_ = now + 1.;
      return;
    }
    // Bounded validation history; weak excitation cannot approve a candidate.
    if (validation_.size() > size_t(config_.horizon))
      validation_.erase(validation_.begin(),
                        validation_.end() - config_.horizon);
    result_.status = "validation needs varied joint motion";
  }
  if (pending_end_ >= 0)
    return;
  if (result_.rows < config_.horizon) {
    result_.status = "collecting";
    return;
  }
  if (now < next_fit_)
    return;
  next_fit_ = now + 1;
  auto d = stack(training_);
  auto info = information(d);
  result_.rank = info.first;
  result_.min_information = info.second;
  if (info.first < basis_.cols()) {
    result_.status = "needs varied joint motion";
    return;
  }
  try {
    result_.candidate = fit(d);
    pending_end_ = now;
    validation_.clear();
    result_.validation_error = -1;
    result_.status = "checking independent measurements";
  } catch (const std::exception &e) {
    result_.status = "fit failed";
    result_.detail = e.what();
  }
}
} // namespace uvms::arm_identification
