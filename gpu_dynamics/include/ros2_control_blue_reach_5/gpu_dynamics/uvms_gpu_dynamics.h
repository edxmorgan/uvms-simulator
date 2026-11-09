#pragma once

// Author: edward morgan

#include <cstdint>

namespace uvms_gpu {

struct VehicleStepDevicePointers {
    const double* x_vehicle;
    const double* vehicle_wrench;
    const double* vehicle_params;
    const double* dt;
    const double* external_wrench;
    double* x_vehicle_next;
};

struct ArmStepDevicePointers {
    const double* x_arm;
    const double* arm_torque;
    const double* dt;
    const double* arm_params;
    const double* ee_mass;
    const double* ee_damping;
    const double* ee_stiffness;
    const double* lock_mask;
    const double* baumgarte_alpha;
    double* x_arm_next;
};

// Full UVMS dynamics is exposed as separate vehicle and arm batch kernels.
void launch_vehicle_step(const VehicleStepDevicePointers& ptrs,
                         int batch_size,
                         int threads_per_block = 256,
                         std::uintptr_t stream_ptr = 0,
                         bool sync = false);

void launch_arm_step(const ArmStepDevicePointers& ptrs,
                     int batch_size,
                     int threads_per_block = 256,
                     std::uintptr_t stream_ptr = 0,
                     bool sync = false);

void launch_prepare_arm_step(const double* x_arm,
                             double* arm_torque,
                             double* lock_mask,
                             int batch_size,
                             int threads_per_block = 256,
                             std::uintptr_t stream_ptr = 0,
                             bool sync = false);

void launch_clamp_arm_state(double* x_arm,
                            int batch_size,
                            int threads_per_block = 256,
                            std::uintptr_t stream_ptr = 0,
                            bool sync = false);

void device_synchronize();

}  // namespace uvms_gpu
