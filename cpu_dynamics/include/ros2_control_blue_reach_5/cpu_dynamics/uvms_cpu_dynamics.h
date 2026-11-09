#pragma once

// Author: edward morgan

namespace uvms_cpu {

struct VehicleStepHostPointers {
    const double* x_vehicle;
    const double* vehicle_wrench;
    const double* vehicle_params;
    const double* dt;
    const double* external_wrench;
    double* x_vehicle_next;
};

struct ArmStepHostPointers {
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

struct UvmsStepHostPointers {
    const double* x_uvms;
    const double* vehicle_wrench;
    const double* arm_torque;
    const double* vehicle_params;
    const double* arm_params;
    const double* dt;
    const double* external_wrench;
    const double* ee_mass;
    const double* ee_damping;
    const double* ee_stiffness;
    const double* lock_mask;
    const double* baumgarte_alpha;
    double* x_uvms_next;
};

void step_vehicle(const VehicleStepHostPointers& ptrs, int batch_size, int max_threads = 0);
void step_arm(const ArmStepHostPointers& ptrs, int batch_size, int max_threads = 0);
void step_uvms(const UvmsStepHostPointers& ptrs, int batch_size, int max_threads = 0);

}  // namespace uvms_cpu
