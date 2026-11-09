// Copyright (C) 2026 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.

#include "ros2_control_blue_reach_5/gpu_dynamics/uvms_gpu_dynamics.h"

#include <cuda_runtime_api.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace uvms_gpu {
namespace {

constexpr int kArmJointCount = 5;
constexpr int kArmStateDim = 10;
constexpr int kLockMaskDim = 4;
constexpr double kJointLockOnDeadband = 0.05;
constexpr double kJointLockOffDeadband = 0.10;
constexpr double kTorqueCurrentZeroEpsilon = 1.0e-9;

__constant__ double kArmPositionMin[kArmJointCount] = {
    1.0, 0.01, 0.01, 0.01, 0.0004};
__constant__ double kArmPositionMax[kArmJointCount] = {
    5.5, 3.4, 3.4, 5.7, 0.0137};
__constant__ double kArmCurrentMax[kArmJointCount] = {
    600.0, 400.0, 400.0, 400.0, 400.0};
__constant__ double kArmMotorKt[kArmJointCount] = {
    90.6, 90.6, 90.6, 50.0, 50.0};
__constant__ double kArmForwardStaticCurrent[kArmJointCount] = {
    43.0, 43.0, 43.0, 10.0, 43.0};
__constant__ double kArmBackwardStaticCurrent[kArmJointCount] = {
    43.0, 43.0, 43.0, 20.0, 43.0};

cudaStream_t stream_from_uintptr(std::uintptr_t stream_ptr) {
    return reinterpret_cast<cudaStream_t>(stream_ptr);
}

void check_cuda(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
}

__device__ double torque_to_current(double kt, double static_current, double torque) {
    if (fabs(torque) <= kTorqueCurrentZeroEpsilon) {
        return 0.0;
    }

    // Match arm_actuator_model::torque_to_current: every nonzero request
    // includes the static-current offset, so the inverse preserves small torque.
    return kt * torque + copysign(static_current, torque);
}

__device__ double current_to_torque(double kt, double static_current, double current) {
    if (fabs(current) <= static_current) {
        return 0.0;
    }
    return (current - copysign(static_current, current)) / kt;
}

__global__ void prepare_arm_step_kernel(const double* x_arm,
                                        double* arm_torque,
                                        double* lock_mask,
                                        int batch_size) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = batch_size * kArmJointCount;
    if (idx >= total) {
        return;
    }

    const int robot = idx / kArmJointCount;
    const int joint = idx - robot * kArmJointCount;
    const int torque_index = robot * kArmJointCount + joint;
    const double tau_cmd = arm_torque[torque_index];
    const double static_current = tau_cmd >= 0.0
                                     ? kArmForwardStaticCurrent[joint]
                                     : kArmBackwardStaticCurrent[joint];
    const double i_cmd = torque_to_current(kArmMotorKt[joint], static_current, tau_cmd);

    double min_current = -kArmCurrentMax[joint];
    double max_current = kArmCurrentMax[joint];
    const double q = x_arm[robot * kArmStateDim + joint];
    if (q < kArmPositionMin[joint]) {
        min_current = 0.0;
    } else if (q > kArmPositionMax[joint]) {
        max_current = 0.0;
    }

    const double i_safe = fmin(fmax(i_cmd, min_current), max_current);
    double tau_safe = 0.0;
    if (fabs(i_safe) > kTorqueCurrentZeroEpsilon) {
        const double safe_static_current = i_safe >= 0.0
                                              ? kArmForwardStaticCurrent[joint]
                                              : kArmBackwardStaticCurrent[joint];
        tau_safe = current_to_torque(kArmMotorKt[joint], safe_static_current, i_safe);
    }
    arm_torque[torque_index] = tau_safe;

    if (joint < kLockMaskDim) {
        const int mask_index = robot * kLockMaskDim + joint;
        const double effort = fabs(tau_safe);
        const bool locked = lock_mask[mask_index] != 0.0;

        if (!locked && effort < kJointLockOnDeadband) {
            lock_mask[mask_index] = 1.0;
        } else if (locked && effort > kJointLockOffDeadband) {
            lock_mask[mask_index] = 0.0;
        }
    }
}

__global__ void clamp_arm_state_kernel(double* x_arm, int batch_size) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = batch_size * kArmJointCount;
    if (idx >= total) {
        return;
    }

    const int robot = idx / kArmJointCount;
    const int joint = idx - robot * kArmJointCount;
    const int q_index = robot * kArmStateDim + joint;
    const int qd_index = robot * kArmStateDim + kArmJointCount + joint;

    double q = x_arm[q_index];
    double qd = x_arm[qd_index];
    if (q < kArmPositionMin[joint]) {
        q = kArmPositionMin[joint];
        if (qd < 0.0) {
            qd = 0.0;
        }
    } else if (q > kArmPositionMax[joint]) {
        q = kArmPositionMax[joint];
        if (qd > 0.0) {
            qd = 0.0;
        }
    }

    x_arm[q_index] = q;
    x_arm[qd_index] = qd;
}

}  // namespace

void launch_prepare_arm_step(const double* x_arm,
                             double* arm_torque,
                             double* lock_mask,
                             int batch_size,
                             int threads_per_block,
                             std::uintptr_t stream_ptr,
                             bool sync) {
    if (batch_size <= 0) {
        return;
    }
    if (x_arm == nullptr || arm_torque == nullptr || lock_mask == nullptr) {
        throw std::invalid_argument("launch_prepare_arm_step received null device pointer");
    }
    const int total = batch_size * kArmJointCount;
    const int blocks = (total + threads_per_block - 1) / threads_per_block;
    const cudaStream_t stream = stream_from_uintptr(stream_ptr);
    prepare_arm_step_kernel<<<blocks, threads_per_block, 0, stream>>>(
        x_arm, arm_torque, lock_mask, batch_size);
    check_cuda(cudaGetLastError(), "prepare_arm_step_kernel launch failed");
    if (sync) {
        check_cuda(cudaStreamSynchronize(stream), "prepare_arm_step_kernel synchronize failed");
    }
}

void launch_clamp_arm_state(double* x_arm,
                            int batch_size,
                            int threads_per_block,
                            std::uintptr_t stream_ptr,
                            bool sync) {
    if (batch_size <= 0) {
        return;
    }
    if (x_arm == nullptr) {
        throw std::invalid_argument("launch_clamp_arm_state received null device pointer");
    }
    const int total = batch_size * kArmJointCount;
    const int blocks = (total + threads_per_block - 1) / threads_per_block;
    const cudaStream_t stream = stream_from_uintptr(stream_ptr);
    clamp_arm_state_kernel<<<blocks, threads_per_block, 0, stream>>>(x_arm, batch_size);
    check_cuda(cudaGetLastError(), "clamp_arm_state_kernel launch failed");
    if (sync) {
        check_cuda(cudaStreamSynchronize(stream), "clamp_arm_state_kernel synchronize failed");
    }
}

}  // namespace uvms_gpu
