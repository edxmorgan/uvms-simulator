#include "ros2_control_blue_reach_5/nominal_arm_parameters.hpp"
#include "ros2_control_blue_reach_5/sim_reach_system_multi_interface.hpp"

namespace ros2_control_blue_reach_5 {
void SimReachSystemMultiInterfaceHardware::clear_identified_model(
    int subsystem) {
  if (subsystem == 0 && identification_ack_[0] == 1.) {
    estimator_nominal_parameters_ = casadi::DM::vertcat(
        pack_manipulator_dynamics(default_manipulator_dynamics()));
    for (int j = 0; j < 81; ++j)
      estimator_model_readback_[j] = double(estimator_nominal_parameters_(j));
    // Preserve the joint-state marginal, but do not keep disturbance estimates
    // whose meaning depended on the discarded dynamics model.
    for (int j = 8; j < 12; ++j) {
      arm_x_est_(j) = 0;
      for (int k = 0; k < 12; ++k)
        arm_P_est_(j, k) = arm_P_est_(k, j) = 0;
      arm_P_est_(j, j) = 1;
    }
    estimated_disturbance_.fill(0);
    disturbance_variance_.fill(1);
    disturbance_covariance_.fill(0);
    for (int j = 0; j < 4; ++j)
      disturbance_covariance_[4 * j + j] = 1;
  }
  if (subsystem == 1)
    gripper_model_readback_ = {300, 400, 0};
  identification_ack_[3 * subsystem] = 0;
  identification_ack_[3 * subsystem + 1] =
      identification_ack_[3 * subsystem + 2] = -1;
}
void SimReachSystemMultiInterfaceHardware::reset_identified_models() {
  for (int j = 0; j < 2; ++j) {
    clear_identified_model(j);
    pending_identified_models_[j].reset();
    identification_epoch_seen_[j] = 0;
    identification_stamp_seen_[j] = 0;
  }
  if (node_frames_interface_)
    identified_model_reset_stamp_ = node_frames_interface_->now();
}
void SimReachSystemMultiInterfaceHardware::receive_identified_model(
    const ArmIdentification &m) {
  std::lock_guard<std::mutex> lock(simulation_state_mutex_);
  if (m.subsystem != "arm" && m.subsystem != "gripper")
    return;
  int index = m.subsystem == "arm" ? 0 : 1;
  const auto stamp = rclcpp::Time(m.stamp);
  const double age = (node_frames_interface_->now() - stamp).seconds();
  if (m.joint != get_hardware_info().gpios[0].name ||
      m.source_generation != sim_generation_ || m.epoch >= (1ULL << 53) ||
      m.epoch < identification_epoch_seen_[index] ||
      stamp < identified_model_reset_stamp_ ||
      stamp.nanoseconds() < identification_stamp_seen_[index] || age < 0 ||
      age > 3)
    return;
  if (m.retained_valid) {
    const bool already_active =
        identification_ack_[3 * index] == 1 &&
        identification_ack_[3 * index + 1] == double(m.epoch) &&
        identification_ack_[3 * index + 2] == m.retained_accepted_time;
    if ((!m.measurements_fresh && !already_active) ||
        !identification_sample_.supported ||
        !std::isfinite(m.retained_accepted_time) ||
        !std::isfinite(m.observation_time) || m.retained_accepted_time < 0 ||
        m.retained_accepted_time > m.observation_time ||
        m.observation_time > identification_sample_.time + 1e-6 ||
        identification_sample_.time - m.observation_time > 3. ||
        m.model_context != identification_sample_.context ||
        !uvms::arm_identification::physical(m.retained_parameters, index == 1))
      return;
    if (identification_ack_[3 * index] == 1 &&
        m.epoch == uint64_t(identification_ack_[3 * index + 1]) &&
        m.retained_accepted_time < identification_ack_[3 * index + 2])
      return;
  }
  identification_epoch_seen_[index] = m.epoch;
  identification_stamp_seen_[index] = stamp.nanoseconds();
  pending_identified_models_[index] = m;
  identified_model_received_[index] = std::chrono::steady_clock::now();
}
void SimReachSystemMultiInterfaceHardware::apply_identified_models() {
  for (int index = 0; index < 2; ++index) {
    const double age =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      identified_model_received_[index])
            .count();
    if (age > 3 || !identification_sample_.supported) {
      clear_identified_model(index);
      pending_identified_models_[index].reset();
      continue;
    }
    if (!pending_identified_models_[index])
      continue;
    const auto m = *pending_identified_models_[index];
    pending_identified_models_[index].reset();
    if (m.source_generation != sim_generation_ || !m.retained_valid ||
        m.model_context != identification_sample_.context) {
      clear_identified_model(index);
      continue;
    }
    if (identification_ack_[3 * index] == 1 &&
        identification_ack_[3 * index + 1] == double(m.epoch) &&
        identification_ack_[3 * index + 2] == m.retained_accepted_time)
      continue;
    const auto &p = m.retained_parameters;
    if (index == 0) {
      auto sample = identification_sample_;
      for (int j = 0; j < 4; ++j)
        sample.position[j] = double(arm_x_est_(j));
      if (!uvms::arm_identification::invertible(p, sample)) {
        RCLCPP_WARN(node_frames_interface_->get_logger(),
                    "Validated arm model has singular articulated inertia at "
                    "current pose; keeping previous EKF model");
        continue;
      }
      auto parameters = casadi::DM::vertcat(
          pack_manipulator_dynamics(default_manipulator_dynamics()));
      for (int j = 0; j < 4; ++j) {
        parameters(j) = p[12 * j];
        for (int k = 0; k < 3; ++k)
          parameters(4 + 3 * j + k) = p[12 * j + 1 + k];
        for (int k = 0; k < 6; ++k)
          parameters(16 + 6 * j + k) = p[12 * j + 4 + k];
        parameters(40 + j) = std::max(0., p[12 * j + 10]);
        parameters(44 + j) = std::max(0., p[12 * j + 11]);
      }
      for (int j = 0; j < 3; ++j)
        parameters(56 + j) = m.model_context[j];
      for (int j = 0; j < 12; ++j)
        parameters(63 + j) = m.model_context[3 + j];
      // Both operations are under the read() state mutex, before prediction.
      // A new fit must not reset joint position/velocity or their covariance.
      for (int j = 8; j < 12; ++j) {
        arm_x_est_(j) = 0;
        for (int k = 0; k < 12; ++k)
          arm_P_est_(j, k) = arm_P_est_(k, j) = 0;
        arm_P_est_(j, j) = 1;
      }
      estimator_nominal_parameters_ = parameters;
      for (int j = 0; j < 81; ++j)
        estimator_model_readback_[j] = double(parameters(j));
    } else {
      Eigen::Vector2d qv(double(x_est_list_[4](0)), double(x_est_list_[4](1)));
      Eigen::Matrix2d P;
      for (int j = 0; j < 2; ++j)
        for (int k = 0; k < 2; ++k)
          P(j, k) = double(P_est_list_[4](j, k));
      const Eigen::Vector3d model(p[0], std::max(0., p[1]), std::max(0., p[2]));
      gripper_observer_.handover(qv, P, model);
      for (int j = 0; j < 3; ++j)
        gripper_model_readback_[j] = model[j];
    }
    identification_ack_[3 * index] = 1;
    identification_ack_[3 * index + 1] = double(m.epoch);
    identification_ack_[3 * index + 2] = m.retained_accepted_time;
  }
}
} // namespace ros2_control_blue_reach_5
