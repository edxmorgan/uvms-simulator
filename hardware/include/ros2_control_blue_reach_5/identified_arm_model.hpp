#pragma once
#include "ros2_control_blue_reach_5/arm_identifier.hpp"
#include <unsupported/Eigen/MatrixFunctions>

namespace uvms::arm_identification {
inline bool physical(const std::vector<double> &parameters, bool gripper) {
  if (parameters.size() != size_t(gripper ? 3 : 48))
    return false;
  for (double p : parameters)
    if (!std::isfinite(p))
      return false;
  if (gripper)
    return parameters[0] >= .001 && parameters[0] <= 10000 &&
           parameters[1] >= -1e-7 && parameters[2] >= -1e-7;
  for (int j = 0; j < 4; ++j) {
    const double *t = parameters.data() + 12 * j;
    Eigen::Matrix3d inertia;
    inertia << t[4], t[7], t[8], t[7], t[5], t[9], t[8], t[9], t[6];
    Eigen::Matrix4d pseudo = Eigen::Matrix4d::Zero();
    pseudo.topLeftCorner<3, 3>() =
        .5 * inertia.trace() * Eigen::Matrix3d::Identity() - inertia;
    for (int i = 0; i < 3; ++i)
      pseudo(i, 3) = pseudo(3, i) = t[1 + i];
    pseudo(3, 3) = t[0];
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(pseudo);
    if (t[0] < .001 - 1e-7 || t[0] > 10 + 1e-7 || t[10] < -1e-7 ||
        t[11] < -1e-7 || eig.info() != Eigen::Success ||
        eig.eigenvalues().minCoeff() <= 1e-9)
      return false;
  }
  return true;
}
inline bool invertible(const std::vector<double> &p, Sample s) {
  if (!physical(p, false))
    return false;
  s.velocity.fill(0);
  const auto zero = regressor(s, Eigen::Vector4d::Zero());
  const Eigen::Map<const Eigen::VectorXd> theta(p.data(), 48);
  Eigen::Matrix4d mass;
  for (int j = 0; j < 4; ++j)
    mass.col(j) = (regressor(s, Eigen::Vector4d::Unit(j)) - zero) * theta;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig((mass + mass.transpose()) *
                                                     .5);
  return mass.allFinite() && (mass - mass.transpose()).norm() < 1e-6 &&
         eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 1e-7;
}
// Dynamics-based gripper filter, with an unmeasured external effort state.
// The previous kinematic filter has no model parameters to update.
struct GripperObserver {
  Eigen::Vector3d x =
      Eigen::Vector3d::Zero(); // position, velocity, external effort
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  Eigen::Vector3d model{300, 400, 0};
  void handover(const Eigen::Vector2d &qv, const Eigen::Matrix2d &P,
                const Eigen::Vector3d &p) {
    x << qv[0], qv[1], 0;
    covariance.setZero();
    covariance.topLeftCorner<2, 2>() = P;
    covariance(2, 2) = 1;
    model = p;
  }
  void update(double dt, double input, const Eigen::Vector2d &measurement,
              const Eigen::Matrix2d &R, bool contact) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(input) ||
        !measurement.allFinite())
      throw std::invalid_argument("Invalid gripper observation");
    if (contact) {
      x.head<2>() = measurement;
      covariance.topRows<2>().setZero();
      covariance.leftCols<2>().setZero();
      covariance.topLeftCorner<2, 2>() = R;
      covariance(2, 2) += .001 * dt;
      return;
    }
    Eigen::Matrix4d dynamics = Eigen::Matrix4d::Zero();
    dynamics(0, 1) = 1;
    dynamics(1, 0) = -model[2] / model[0];
    dynamics(1, 1) = -model[1] / model[0];
    dynamics(1, 2) = dynamics(1, 3) = 1 / model[0];
    const Eigen::Matrix4d transition = (dynamics * dt).exp();
    const Eigen::Matrix3d F = transition.topLeftCorner<3, 3>();
    x = F * x + transition.topRightCorner<3, 1>() * input;
    covariance = F * covariance * F.transpose();
    covariance.diagonal() += Eigen::Vector3d(1e-8, 1e-6, .001) * dt;
    const Eigen::Matrix2d S = covariance.topLeftCorner<2, 2>() + R;
    const Eigen::Matrix<double, 3, 2> K =
        S.ldlt().solve(covariance.leftCols<2>().transpose()).transpose();
    x += K * (measurement - x.head<2>());
    Eigen::Matrix3d residual = Eigen::Matrix3d::Identity();
    residual.leftCols<2>() -= K;
    covariance =
        residual * covariance * residual.transpose() + K * R * K.transpose();
    covariance = (covariance + covariance.transpose()).eval() * .5;
    if (!x.allFinite() || !covariance.allFinite())
      throw std::runtime_error("Nonfinite gripper posterior");
  }
  Eigen::Matrix3d kinematic_jacobian() const {
    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
    J.row(2) << -model[2] / model[0], -model[1] / model[0], 1 / model[0];
    return J;
  }
  double acceleration(double input) const {
    return (input + x[2] - model[1] * x[1] - model[2] * x[0]) / model[0];
  }
};
} // namespace uvms::arm_identification
