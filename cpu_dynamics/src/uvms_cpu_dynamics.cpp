// Author: edward morgan

#include "ros2_control_blue_reach_5/cpu_dynamics/uvms_cpu_dynamics.h"

#include <array>
#include <stdexcept>

extern "C" {
int Vnext_reg(const float** arg, float** res, long long int* iw, float* w, int mem);
int Mnext_reg(const float** arg, float** res, long long int* iw, float* w, int mem);
}

namespace uvms_cpu {
namespace {
constexpr int kVehicleStateDim = 12;
constexpr int kVehicleWrenchDim = 6;
constexpr int kVehicleParamDim = 33;
constexpr int kArmStateDim = 10;
constexpr int kArmTorqueDim = 5;
constexpr int kArmParamDim = 81;
constexpr int kLockMaskDim = 4;
constexpr int kUvmsStateDim = kVehicleStateDim + kArmStateDim;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::invalid_argument(message);
    }
}

}  // namespace

void step_vehicle(const VehicleStepHostPointers& ptrs, int batch_size) {
    require(batch_size > 0, "batch_size must be positive");
    require(ptrs.x_vehicle != nullptr, "x_vehicle must not be null");
    require(ptrs.vehicle_wrench != nullptr, "vehicle_wrench must not be null");
    require(ptrs.vehicle_params != nullptr, "vehicle_params must not be null");
    require(ptrs.dt != nullptr, "dt must not be null");
    require(ptrs.external_wrench != nullptr, "external_wrench must not be null");
    require(ptrs.x_vehicle_next != nullptr, "x_vehicle_next must not be null");

    for (int i = 0; i < batch_size; ++i) {
        const float* arg[5] = {
            ptrs.x_vehicle + kVehicleStateDim * i,
            ptrs.vehicle_wrench + kVehicleWrenchDim * i,
            ptrs.vehicle_params + kVehicleParamDim * i,
            ptrs.dt + i,
            ptrs.external_wrench + kVehicleWrenchDim * i,
        };
        float* res[1] = {ptrs.x_vehicle_next + kVehicleStateDim * i};
        std::array<long long int, 1> iw{};
        std::array<float, 830> w{};
        Vnext_reg(arg, res, iw.data(), w.data(), 0);
    }
}

void step_arm(const ArmStepHostPointers& ptrs, int batch_size) {
    require(batch_size > 0, "batch_size must be positive");
    require(ptrs.x_arm != nullptr, "x_arm must not be null");
    require(ptrs.arm_torque != nullptr, "arm_torque must not be null");
    require(ptrs.dt != nullptr, "dt must not be null");
    require(ptrs.arm_params != nullptr, "arm_params must not be null");
    require(ptrs.ee_mass != nullptr, "ee_mass must not be null");
    require(ptrs.ee_damping != nullptr, "ee_damping must not be null");
    require(ptrs.ee_stiffness != nullptr, "ee_stiffness must not be null");
    require(ptrs.lock_mask != nullptr, "lock_mask must not be null");
    require(ptrs.baumgarte_alpha != nullptr, "baumgarte_alpha must not be null");
    require(ptrs.x_arm_next != nullptr, "x_arm_next must not be null");

    for (int i = 0; i < batch_size; ++i) {
        const float* arg[9] = {
            ptrs.x_arm + kArmStateDim * i,
            ptrs.arm_torque + kArmTorqueDim * i,
            ptrs.dt + i,
            ptrs.arm_params + kArmParamDim * i,
            ptrs.ee_mass + i,
            ptrs.ee_damping + i,
            ptrs.ee_stiffness + i,
            ptrs.lock_mask + kLockMaskDim * i,
            ptrs.baumgarte_alpha + i,
        };
        float* res[1] = {ptrs.x_arm_next + kArmStateDim * i};
        std::array<long long int, 1> iw{};
        std::array<float, 1073> w{};
        Mnext_reg(arg, res, iw.data(), w.data(), 0);
    }
}

void step_uvms(const UvmsStepHostPointers& ptrs, int batch_size) {
    require(batch_size > 0, "batch_size must be positive");
    require(ptrs.x_uvms != nullptr, "x_uvms must not be null");
    require(ptrs.x_uvms_next != nullptr, "x_uvms_next must not be null");

    for (int i = 0; i < batch_size; ++i) {
        const float* x = ptrs.x_uvms + kUvmsStateDim * i;
        float* x_next = ptrs.x_uvms_next + kUvmsStateDim * i;

        const float* vehicle_arg[5] = {
            x,
            ptrs.vehicle_wrench + kVehicleWrenchDim * i,
            ptrs.vehicle_params + kVehicleParamDim * i,
            ptrs.dt + i,
            ptrs.external_wrench + kVehicleWrenchDim * i,
        };
        float* vehicle_res[1] = {x_next};
        std::array<long long int, 1> vehicle_iw{};
        std::array<float, 830> vehicle_w{};
        Vnext_reg(vehicle_arg, vehicle_res, vehicle_iw.data(), vehicle_w.data(), 0);

        const float* arm_arg[9] = {
            x + kVehicleStateDim,
            ptrs.arm_torque + kArmTorqueDim * i,
            ptrs.dt + i,
            ptrs.arm_params + kArmParamDim * i,
            ptrs.ee_mass + i,
            ptrs.ee_damping + i,
            ptrs.ee_stiffness + i,
            ptrs.lock_mask + kLockMaskDim * i,
            ptrs.baumgarte_alpha + i,
        };
        float* arm_res[1] = {x_next + kVehicleStateDim};
        std::array<long long int, 1> arm_iw{};
        std::array<float, 1073> arm_w{};
        Mnext_reg(arm_arg, arm_res, arm_iw.data(), arm_w.data(), 0);
    }
}

}  // namespace uvms_cpu
