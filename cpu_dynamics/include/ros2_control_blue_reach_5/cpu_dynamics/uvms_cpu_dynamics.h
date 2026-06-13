#pragma once

// Author: edward morgan

namespace uvms_cpu {

struct VehicleStepHostPointers {
    const float* x_vehicle;
    const float* vehicle_wrench;
    const float* vehicle_params;
    const float* dt;
    const float* external_wrench;
    float* x_vehicle_next;
};

struct ArmStepHostPointers {
    const float* x_arm;
    const float* arm_torque;
    const float* dt;
    const float* arm_params;
    const float* ee_mass;
    const float* ee_damping;
    const float* ee_stiffness;
    const float* lock_mask;
    const float* baumgarte_alpha;
    float* x_arm_next;
};

struct UvmsStepHostPointers {
    const float* x_uvms;
    const float* vehicle_wrench;
    const float* arm_torque;
    const float* vehicle_params;
    const float* arm_params;
    const float* dt;
    const float* external_wrench;
    const float* ee_mass;
    const float* ee_damping;
    const float* ee_stiffness;
    const float* lock_mask;
    const float* baumgarte_alpha;
    float* x_uvms_next;
};

void step_vehicle(const VehicleStepHostPointers& ptrs, int batch_size, int max_threads = 0);
void step_arm(const ArmStepHostPointers& ptrs, int batch_size, int max_threads = 0);
void step_uvms(const UvmsStepHostPointers& ptrs, int batch_size, int max_threads = 0);

}  // namespace uvms_cpu
