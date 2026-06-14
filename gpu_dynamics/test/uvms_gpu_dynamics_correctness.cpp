// Author: edward morgan

#include "ros2_control_blue_reach_5/cpu_dynamics/uvms_cpu_dynamics.h"
#include "ros2_control_blue_reach_5/gpu_dynamics/uvms_gpu_dynamics.h"

#include "uvms_dynamics_test_data.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kBatchSize = 32;
constexpr int kRolloutSteps = 100;
constexpr float kAbsTolerance = 1.0e-3F;
constexpr float kRelTolerance = 1.0e-3F;

void check_cuda(cudaError_t err, const char* context) {
    if (err == cudaSuccess) {
        return;
    }
    throw std::runtime_error(std::string(context) + ": " + cudaGetErrorString(err));
}

struct DeviceBuffer {
    float* ptr = nullptr;
    std::size_t count = 0;

    explicit DeviceBuffer(std::size_t n) : count(n) {
        check_cuda(cudaMalloc(&ptr, count * sizeof(float)), "cudaMalloc failed");
    }

    DeviceBuffer(const DeviceBuffer&) = delete;
    DeviceBuffer& operator=(const DeviceBuffer&) = delete;

    ~DeviceBuffer() {
        if (ptr != nullptr) {
            cudaFree(ptr);
        }
    }

    void copy_from(const std::vector<float>& host) {
        if (host.size() != count) {
            throw std::invalid_argument("copy_from size mismatch");
        }
        check_cuda(
            cudaMemcpy(ptr, host.data(), count * sizeof(float), cudaMemcpyHostToDevice),
            "cudaMemcpy host-to-device failed");
    }

    std::vector<float> copy_to_host() const {
        std::vector<float> host(count, 0.0F);
        check_cuda(
            cudaMemcpy(host.data(), ptr, count * sizeof(float), cudaMemcpyDeviceToHost),
            "cudaMemcpy device-to-host failed");
        return host;
    }
};

void assert_close(const char* name, const std::vector<float>& gpu, const std::vector<float>& cpu) {
    int index = 0;
    const float err = uvms_dynamics_test::max_error(gpu, cpu, &index);
    const float scale = std::max(std::abs(gpu[static_cast<std::size_t>(index)]), std::abs(cpu[static_cast<std::size_t>(index)]));
    const float allowed = kAbsTolerance + kRelTolerance * scale;
    std::cout << name << " max_abs_error=" << err << " index=" << index
              << " gpu=" << gpu[static_cast<std::size_t>(index)]
              << " cpu=" << cpu[static_cast<std::size_t>(index)]
              << " tolerance=" << allowed << '\n';
    if (!(err <= allowed)) {
        throw std::runtime_error(std::string(name) + " GPU/CPU mismatch");
    }
}

void run_rollout_case(const char* name, const std::vector<float>& lock_mask) {
    using namespace uvms_dynamics_test;

    const std::vector<float> vehicle_wrench = make_matrix(kBatchSize, kVehicleWrenchDim, 0.01F);
    const std::vector<float> arm_torque = make_matrix(kBatchSize, kArmTorqueDim, 0.01F);
    const std::vector<float> vehicle_params = tile(default_vehicle_params(), kBatchSize);
    const std::vector<float> arm_params = tile(default_arm_params(), kBatchSize);
    const std::vector<float> dt(static_cast<std::size_t>(kBatchSize), 0.01F);
    const std::vector<float> external_wrench = filled(kBatchSize, kExternalWrenchDim, 0.0F);
    const std::vector<float> ee_mass(static_cast<std::size_t>(kBatchSize), 300.0F);
    const std::vector<float> ee_damping(static_cast<std::size_t>(kBatchSize), 400.0F);
    const std::vector<float> ee_stiffness(static_cast<std::size_t>(kBatchSize), 0.0F);
    const std::vector<float> baumgarte_alpha(static_cast<std::size_t>(kBatchSize), 200.0F);

    if (lock_mask.size() != static_cast<std::size_t>(kBatchSize) * kLockMaskDim) {
        throw std::invalid_argument("lock_mask size mismatch");
    }

    std::vector<float> cpu_state = make_uvms_state(kBatchSize);
    std::vector<float> cpu_next(static_cast<std::size_t>(kBatchSize) * kUvmsStateDim, 0.0F);

    std::vector<float> x_vehicle;
    std::vector<float> x_arm;
    split_uvms_state(cpu_state, kBatchSize, &x_vehicle, &x_arm);

    DeviceBuffer d_vehicle_a(x_vehicle.size());
    DeviceBuffer d_vehicle_b(x_vehicle.size());
    DeviceBuffer d_arm_a(x_arm.size());
    DeviceBuffer d_arm_b(x_arm.size());
    DeviceBuffer d_vehicle_wrench(vehicle_wrench.size());
    DeviceBuffer d_vehicle_params(vehicle_params.size());
    DeviceBuffer d_dt(dt.size());
    DeviceBuffer d_external_wrench(external_wrench.size());
    DeviceBuffer d_arm_torque(arm_torque.size());
    DeviceBuffer d_arm_params(arm_params.size());
    DeviceBuffer d_ee_mass(ee_mass.size());
    DeviceBuffer d_ee_damping(ee_damping.size());
    DeviceBuffer d_ee_stiffness(ee_stiffness.size());
    DeviceBuffer d_lock_mask(lock_mask.size());
    DeviceBuffer d_baumgarte_alpha(baumgarte_alpha.size());

    d_vehicle_a.copy_from(x_vehicle);
    d_arm_a.copy_from(x_arm);
    d_vehicle_wrench.copy_from(vehicle_wrench);
    d_vehicle_params.copy_from(vehicle_params);
    d_dt.copy_from(dt);
    d_external_wrench.copy_from(external_wrench);
    d_arm_torque.copy_from(arm_torque);
    d_arm_params.copy_from(arm_params);
    d_ee_mass.copy_from(ee_mass);
    d_ee_damping.copy_from(ee_damping);
    d_ee_stiffness.copy_from(ee_stiffness);
    d_lock_mask.copy_from(lock_mask);
    d_baumgarte_alpha.copy_from(baumgarte_alpha);

    float* d_vehicle_in = d_vehicle_a.ptr;
    float* d_vehicle_out = d_vehicle_b.ptr;
    float* d_arm_in = d_arm_a.ptr;
    float* d_arm_out = d_arm_b.ptr;

    for (int step = 0; step < kRolloutSteps; ++step) {
        uvms_cpu::UvmsStepHostPointers cpu_ptrs{
            cpu_state.data(), vehicle_wrench.data(), arm_torque.data(), vehicle_params.data(), arm_params.data(),
            dt.data(), external_wrench.data(), ee_mass.data(), ee_damping.data(), ee_stiffness.data(),
            lock_mask.data(), baumgarte_alpha.data(), cpu_next.data()};
        uvms_cpu::step_uvms(cpu_ptrs, kBatchSize, 1);
        cpu_state.swap(cpu_next);

        uvms_gpu::VehicleStepDevicePointers vehicle_ptrs{
            d_vehicle_in, d_vehicle_wrench.ptr, d_vehicle_params.ptr, d_dt.ptr,
            d_external_wrench.ptr, d_vehicle_out};
        uvms_gpu::ArmStepDevicePointers arm_ptrs{
            d_arm_in, d_arm_torque.ptr, d_dt.ptr, d_arm_params.ptr, d_ee_mass.ptr,
            d_ee_damping.ptr, d_ee_stiffness.ptr, d_lock_mask.ptr, d_baumgarte_alpha.ptr,
            d_arm_out};
        uvms_gpu::launch_vehicle_step(vehicle_ptrs, kBatchSize);
        uvms_gpu::launch_arm_step(arm_ptrs, kBatchSize);
        std::swap(d_vehicle_in, d_vehicle_out);
        std::swap(d_arm_in, d_arm_out);
    }
    uvms_gpu::device_synchronize();

    const std::vector<float> gpu_vehicle(
        d_vehicle_in == d_vehicle_a.ptr ? d_vehicle_a.copy_to_host() : d_vehicle_b.copy_to_host());
    const std::vector<float> gpu_arm(
        d_arm_in == d_arm_a.ptr ? d_arm_a.copy_to_host() : d_arm_b.copy_to_host());
    const std::vector<float> gpu_state = combine_uvms_state(gpu_vehicle, gpu_arm, kBatchSize);

    assert_close(name, gpu_state, cpu_state);
}

}  // namespace

int main() {
    using namespace uvms_dynamics_test;

    try {
        run_rollout_case("uvms_rollout_zero_lock", filled(kBatchSize, kLockMaskDim, 0.0F));

        std::vector<float> mixed_lock = filled(kBatchSize, kLockMaskDim, 0.0F);
        for (int i = 0; i < kBatchSize; ++i) {
            mixed_lock[static_cast<std::size_t>(i) * kLockMaskDim + (i % kLockMaskDim)] =
                (i % 3 == 0) ? 1.0F : 0.0F;
        }
        run_rollout_case("uvms_rollout_mixed_lock", mixed_lock);

        std::cout << "uvms_gpu_dynamics_correctness passed batch_size=" << kBatchSize
                  << " rollout_steps=" << kRolloutSteps << " cases=2" << '\n';
    } catch (const std::exception& exc) {
        std::cerr << "uvms_gpu_dynamics_correctness: " << exc.what() << '\n';
        return 1;
    }
    return 0;
}
