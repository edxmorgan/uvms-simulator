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
constexpr float kJointLockOnDeadband = 0.05F;
constexpr float kJointLockOffDeadband = 0.10F;
constexpr float kTorqueCurrentZeroEpsilon = 1.0e-9F;

__constant__ float kArmPositionMin[kArmJointCount] = {
    1.0F, 0.01F, 0.01F, 0.01F, 0.0004F};
__constant__ float kArmPositionMax[kArmJointCount] = {
    5.5F, 3.4F, 3.4F, 5.7F, 0.0137F};
__constant__ float kArmCurrentMax[kArmJointCount] = {
    600.0F, 400.0F, 400.0F, 400.0F, 400.0F};
__constant__ float kArmMotorKt[kArmJointCount] = {
    90.6F, 90.6F, 90.6F, 50.0F, 50.0F};
__constant__ float kArmForwardStaticCurrent[kArmJointCount] = {
    43.0F, 43.0F, 43.0F, 10.0F, 43.0F};
__constant__ float kArmBackwardStaticCurrent[kArmJointCount] = {
    43.0F, 43.0F, 43.0F, 20.0F, 43.0F};

cudaStream_t stream_from_uintptr(std::uintptr_t stream_ptr) {
    return reinterpret_cast<cudaStream_t>(stream_ptr);
}

void check_cuda(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
}

__device__ float torque_to_current(float kt, float static_current, float torque) {
    if (fabsf(torque) <= kTorqueCurrentZeroEpsilon) {
        return 0.0F;
    }

    float current = kt * torque;
    if (fabsf(torque) >= static_current / kt) {
        current += copysignf(static_current, torque);
    }
    return current;
}

__device__ float current_to_torque(float kt, float static_current, float current) {
    if (fabsf(current) <= static_current) {
        return 0.0F;
    }
    return (current - copysignf(static_current, current)) / kt;
}

__global__ void prepare_arm_step_kernel(const float* x_arm,
                                        float* arm_torque,
                                        float* lock_mask,
                                        int batch_size) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = batch_size * kArmJointCount;
    if (idx >= total) {
        return;
    }

    const int robot = idx / kArmJointCount;
    const int joint = idx - robot * kArmJointCount;
    const int torque_index = robot * kArmJointCount + joint;
    const float tau_cmd = arm_torque[torque_index];
    const float static_current = tau_cmd >= 0.0F
                                     ? kArmForwardStaticCurrent[joint]
                                     : kArmBackwardStaticCurrent[joint];
    const float i_cmd = torque_to_current(kArmMotorKt[joint], static_current, tau_cmd);

    float min_current = -kArmCurrentMax[joint];
    float max_current = kArmCurrentMax[joint];
    const float q = x_arm[robot * kArmStateDim + joint];
    if (q < kArmPositionMin[joint]) {
        min_current = 0.0F;
    } else if (q > kArmPositionMax[joint]) {
        max_current = 0.0F;
    }

    const float i_safe = fminf(fmaxf(i_cmd, min_current), max_current);
    float tau_safe = 0.0F;
    if (fabsf(i_safe) > kTorqueCurrentZeroEpsilon) {
        const float safe_static_current = i_safe >= 0.0F
                                              ? kArmForwardStaticCurrent[joint]
                                              : kArmBackwardStaticCurrent[joint];
        tau_safe = current_to_torque(kArmMotorKt[joint], safe_static_current, i_safe);
    }
    arm_torque[torque_index] = tau_safe;

    if (joint < kLockMaskDim) {
        const int mask_index = robot * kLockMaskDim + joint;
        const float effort = fabsf(tau_safe);
        const bool locked = lock_mask[mask_index] != 0.0F;

        if (!locked && effort < kJointLockOnDeadband) {
            lock_mask[mask_index] = 1.0F;
        } else if (locked && effort > kJointLockOffDeadband) {
            lock_mask[mask_index] = 0.0F;
        }
    }
}

__global__ void clamp_arm_state_kernel(float* x_arm, int batch_size) {
    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const int total = batch_size * kArmJointCount;
    if (idx >= total) {
        return;
    }

    const int robot = idx / kArmJointCount;
    const int joint = idx - robot * kArmJointCount;
    const int q_index = robot * kArmStateDim + joint;
    const int qd_index = robot * kArmStateDim + kArmJointCount + joint;

    float q = x_arm[q_index];
    float qd = x_arm[qd_index];
    if (q < kArmPositionMin[joint]) {
        q = kArmPositionMin[joint];
        if (qd < 0.0F) {
            qd = 0.0F;
        }
    } else if (q > kArmPositionMax[joint]) {
        q = kArmPositionMax[joint];
        if (qd > 0.0F) {
            qd = 0.0F;
        }
    }

    x_arm[q_index] = q;
    x_arm[qd_index] = qd;
}

}  // namespace

void launch_prepare_arm_step(const float* x_arm,
                             float* arm_torque,
                             float* lock_mask,
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

void launch_clamp_arm_state(float* x_arm,
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
