// Author: edward morgan

#include "ros2_control_blue_reach_5/cpu_dynamics/uvms_cpu_dynamics.h"
#include "ros2_control_blue_reach_5/gpu_dynamics/uvms_gpu_dynamics.h"

#include "uvms_dynamics_test_data.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void check_cuda(cudaError_t err, const char* what) {
    if (err == cudaSuccess) {
        return;
    }
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
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

    void copy_from(const std::vector<float>& values) {
        if (values.size() != count) {
            throw std::invalid_argument("host/device buffer size mismatch");
        }
        check_cuda(
            cudaMemcpy(ptr, values.data(), count * sizeof(float), cudaMemcpyHostToDevice),
            "cudaMemcpy host-to-device failed");
    }
};

void print_usage(const char* argv0) {
    std::cerr
        << "usage: " << argv0
        << " [--sizes 128,512,1024] [--iters N] [--warmup N] [--threads N] [--cpu-threads N]"
        << " [--mode full|full_parallel|vehicle|arm|all]\n";
}

std::vector<int> parse_sizes(const std::string& value) {
    std::vector<int> sizes;
    std::size_t start = 0;
    while (start < value.size()) {
        const std::size_t comma = value.find(',', start);
        const std::string token = value.substr(
            start, comma == std::string::npos ? std::string::npos : comma - start);
        sizes.push_back(std::stoi(token));
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    sizes.erase(
        std::remove_if(sizes.begin(), sizes.end(), [](int n) { return n <= 0; }),
        sizes.end());
    return sizes;
}

struct Args {
    std::vector<int> sizes = {128, 512, 1024, 2048, 4096, 8192, 16384};
    int warmup = 10;
    int iters = 100;
    int threads = 256;
    int cpu_threads = 0;
    std::string mode = "full";
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto need_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(std::string(name) + " requires a value");
            }
            return argv[++i];
        };

        if (key == "--sizes") {
            args.sizes = parse_sizes(need_value("--sizes"));
        } else if (key == "--iters") {
            args.iters = std::stoi(need_value("--iters"));
        } else if (key == "--warmup") {
            args.warmup = std::stoi(need_value("--warmup"));
        } else if (key == "--threads") {
            args.threads = std::stoi(need_value("--threads"));
        } else if (key == "--cpu-threads") {
            args.cpu_threads = std::stoi(need_value("--cpu-threads"));
        } else if (key == "--mode") {
            args.mode = need_value("--mode");
        } else if (key == "-h" || key == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + key);
        }
    }

    if (args.sizes.empty() || args.iters <= 0 || args.warmup < 0 || args.threads <= 0 || args.cpu_threads < 0) {
        throw std::invalid_argument("invalid benchmark arguments");
    }
    if (args.mode != "full" && args.mode != "full_parallel" &&
        args.mode != "vehicle" && args.mode != "arm" && args.mode != "all") {
        throw std::invalid_argument("invalid --mode; expected full, full_parallel, vehicle, arm, or all");
    }
    return args;
}

struct MatchedCase {
    std::vector<float> x_uvms;
    std::vector<float> x_vehicle;
    std::vector<float> x_arm;
    std::vector<float> vehicle_wrench;
    std::vector<float> arm_torque;
    std::vector<float> vehicle_params;
    std::vector<float> arm_params;
    std::vector<float> dt;
    std::vector<float> external_wrench;
    std::vector<float> ee_mass;
    std::vector<float> ee_damping;
    std::vector<float> ee_stiffness;
    std::vector<float> lock_mask;
    std::vector<float> baumgarte_alpha;
};

MatchedCase make_case(int batch_size) {
    using namespace uvms_dynamics_test;

    MatchedCase data;
    data.x_uvms = make_uvms_state(batch_size);
    split_uvms_state(data.x_uvms, batch_size, &data.x_vehicle, &data.x_arm);
    data.vehicle_wrench = make_matrix(batch_size, kVehicleWrenchDim, 0.01F);
    data.arm_torque = make_matrix(batch_size, kArmTorqueDim, 0.01F);
    data.vehicle_params = tile(default_vehicle_params(), batch_size);
    data.arm_params = tile(default_arm_params(), batch_size);
    data.dt.assign(static_cast<std::size_t>(batch_size), 0.01F);
    data.external_wrench = filled(batch_size, kExternalWrenchDim, 0.0F);
    data.ee_mass.assign(static_cast<std::size_t>(batch_size), 300.0F);
    data.ee_damping.assign(static_cast<std::size_t>(batch_size), 400.0F);
    data.ee_stiffness.assign(static_cast<std::size_t>(batch_size), 0.0F);
    data.lock_mask = filled(batch_size, kLockMaskDim, 0.0F);
    data.baumgarte_alpha.assign(static_cast<std::size_t>(batch_size), 200.0F);
    return data;
}

double run_cpu_size(int batch_size, const Args& args, const MatchedCase& data, const std::string& mode) {
    std::vector<float> cpu_uvms_next(
        static_cast<std::size_t>(batch_size) * uvms_dynamics_test::kUvmsStateDim, 0.0F);
    std::vector<float> cpu_vehicle_next(
        static_cast<std::size_t>(batch_size) * uvms_dynamics_test::kVehicleStateDim, 0.0F);
    std::vector<float> cpu_arm_next(
        static_cast<std::size_t>(batch_size) * uvms_dynamics_test::kArmStateDim, 0.0F);
    uvms_cpu::UvmsStepHostPointers cpu_ptrs{
        data.x_uvms.data(), data.vehicle_wrench.data(), data.arm_torque.data(),
        data.vehicle_params.data(), data.arm_params.data(), data.dt.data(),
        data.external_wrench.data(), data.ee_mass.data(), data.ee_damping.data(),
        data.ee_stiffness.data(), data.lock_mask.data(), data.baumgarte_alpha.data(),
        cpu_uvms_next.data()};
    uvms_cpu::VehicleStepHostPointers vehicle_ptrs{
        data.x_vehicle.data(), data.vehicle_wrench.data(), data.vehicle_params.data(),
        data.dt.data(), data.external_wrench.data(), cpu_vehicle_next.data()};
    uvms_cpu::ArmStepHostPointers arm_ptrs{
        data.x_arm.data(), data.arm_torque.data(), data.dt.data(), data.arm_params.data(),
        data.ee_mass.data(), data.ee_damping.data(), data.ee_stiffness.data(),
        data.lock_mask.data(), data.baumgarte_alpha.data(), cpu_arm_next.data()};
    auto step = [&]() {
        if (mode == "full" || mode == "full_parallel") {
            uvms_cpu::step_uvms(cpu_ptrs, batch_size, args.cpu_threads);
        } else if (mode == "vehicle") {
            uvms_cpu::step_vehicle(vehicle_ptrs, batch_size, args.cpu_threads);
        } else {
            uvms_cpu::step_arm(arm_ptrs, batch_size, args.cpu_threads);
        }
    };

    for (int i = 0; i < args.warmup; ++i) {
        step();
    }

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < args.iters; ++i) {
        step();
    }
    const auto stop = std::chrono::steady_clock::now();
    const std::chrono::duration<double, std::milli> elapsed = stop - start;
    return elapsed.count() / static_cast<double>(args.iters);
}

double run_gpu_size(int batch_size, const Args& args, const MatchedCase& data, const std::string& mode) {
    DeviceBuffer x_vehicle(data.x_vehicle.size());
    DeviceBuffer vehicle_wrench(data.vehicle_wrench.size());
    DeviceBuffer vehicle_params(data.vehicle_params.size());
    DeviceBuffer dt(data.dt.size());
    DeviceBuffer external_wrench(data.external_wrench.size());
    DeviceBuffer x_vehicle_next(data.x_vehicle.size());

    DeviceBuffer x_arm(data.x_arm.size());
    DeviceBuffer arm_torque(data.arm_torque.size());
    DeviceBuffer arm_params(data.arm_params.size());
    DeviceBuffer ee_mass(data.ee_mass.size());
    DeviceBuffer ee_damping(data.ee_damping.size());
    DeviceBuffer ee_stiffness(data.ee_stiffness.size());
    DeviceBuffer lock_mask(data.lock_mask.size());
    DeviceBuffer baumgarte_alpha(data.baumgarte_alpha.size());
    DeviceBuffer x_arm_next(data.x_arm.size());

    x_vehicle.copy_from(data.x_vehicle);
    vehicle_wrench.copy_from(data.vehicle_wrench);
    vehicle_params.copy_from(data.vehicle_params);
    dt.copy_from(data.dt);
    external_wrench.copy_from(data.external_wrench);

    x_arm.copy_from(data.x_arm);
    arm_torque.copy_from(data.arm_torque);
    arm_params.copy_from(data.arm_params);
    ee_mass.copy_from(data.ee_mass);
    ee_damping.copy_from(data.ee_damping);
    ee_stiffness.copy_from(data.ee_stiffness);
    lock_mask.copy_from(data.lock_mask);
    baumgarte_alpha.copy_from(data.baumgarte_alpha);

    uvms_gpu::VehicleStepDevicePointers vehicle_ptrs{
        x_vehicle.ptr,
        vehicle_wrench.ptr,
        vehicle_params.ptr,
        dt.ptr,
        external_wrench.ptr,
        x_vehicle_next.ptr,
    };
    uvms_gpu::ArmStepDevicePointers arm_ptrs{
        x_arm.ptr,
        arm_torque.ptr,
        dt.ptr,
        arm_params.ptr,
        ee_mass.ptr,
        ee_damping.ptr,
        ee_stiffness.ptr,
        lock_mask.ptr,
        baumgarte_alpha.ptr,
        x_arm_next.ptr,
    };

    cudaEvent_t start{};
    cudaEvent_t stop{};
    cudaEvent_t vehicle_done{};
    cudaEvent_t arm_done{};
    cudaStream_t vehicle_stream{};
    cudaStream_t arm_stream{};
    cudaStream_t timer_stream{};
    check_cuda(cudaEventCreate(&start), "cudaEventCreate start failed");
    check_cuda(cudaEventCreate(&stop), "cudaEventCreate stop failed");
    check_cuda(cudaEventCreate(&vehicle_done), "cudaEventCreate vehicle_done failed");
    check_cuda(cudaEventCreate(&arm_done), "cudaEventCreate arm_done failed");
    if (mode == "full_parallel") {
        check_cuda(cudaStreamCreateWithFlags(&vehicle_stream, cudaStreamNonBlocking), "cudaStreamCreate vehicle failed");
        check_cuda(cudaStreamCreateWithFlags(&arm_stream, cudaStreamNonBlocking), "cudaStreamCreate arm failed");
        check_cuda(cudaStreamCreateWithFlags(&timer_stream, cudaStreamNonBlocking), "cudaStreamCreate timer failed");
    }

    auto step = [&]() {
        const std::uintptr_t vehicle_stream_ptr =
            mode == "full_parallel" ? reinterpret_cast<std::uintptr_t>(vehicle_stream) : 0u;
        const std::uintptr_t arm_stream_ptr =
            mode == "full_parallel" ? reinterpret_cast<std::uintptr_t>(arm_stream) : 0u;
        if (mode == "full_parallel") {
            uvms_gpu::launch_vehicle_step(vehicle_ptrs, batch_size, args.threads, vehicle_stream_ptr);
        } else if (mode == "full" || mode == "vehicle") {
            uvms_gpu::launch_vehicle_step(vehicle_ptrs, batch_size, args.threads);
        }
        if (mode == "full_parallel") {
            uvms_gpu::launch_arm_step(arm_ptrs, batch_size, args.threads, arm_stream_ptr);
        } else if (mode == "full" || mode == "arm") {
            uvms_gpu::launch_arm_step(arm_ptrs, batch_size, args.threads);
        }
    };

    for (int i = 0; i < args.warmup; ++i) {
        step();
    }
    check_cuda(cudaDeviceSynchronize(), "warmup synchronize failed");

    if (mode == "full_parallel") {
        check_cuda(cudaEventRecord(start, timer_stream), "cudaEventRecord start failed");
        check_cuda(cudaStreamWaitEvent(vehicle_stream, start), "cudaStreamWaitEvent vehicle start failed");
        check_cuda(cudaStreamWaitEvent(arm_stream, start), "cudaStreamWaitEvent arm start failed");
        for (int i = 0; i < args.iters; ++i) {
            step();
        }
        check_cuda(cudaEventRecord(vehicle_done, vehicle_stream), "cudaEventRecord vehicle_done failed");
        check_cuda(cudaEventRecord(arm_done, arm_stream), "cudaEventRecord arm_done failed");
        check_cuda(cudaStreamWaitEvent(timer_stream, vehicle_done), "cudaStreamWaitEvent vehicle_done failed");
        check_cuda(cudaStreamWaitEvent(timer_stream, arm_done), "cudaStreamWaitEvent arm_done failed");
        check_cuda(cudaEventRecord(stop, timer_stream), "cudaEventRecord stop failed");
    } else {
        check_cuda(cudaEventRecord(start), "cudaEventRecord start failed");
        for (int i = 0; i < args.iters; ++i) {
            step();
        }
        check_cuda(cudaEventRecord(stop), "cudaEventRecord stop failed");
    }
    check_cuda(cudaEventSynchronize(stop), "cudaEventSynchronize stop failed");

    float elapsed_ms = 0.0F;
    check_cuda(cudaEventElapsedTime(&elapsed_ms, start, stop), "cudaEventElapsedTime failed");
    check_cuda(cudaEventDestroy(start), "cudaEventDestroy start failed");
    check_cuda(cudaEventDestroy(stop), "cudaEventDestroy stop failed");
    check_cuda(cudaEventDestroy(vehicle_done), "cudaEventDestroy vehicle_done failed");
    check_cuda(cudaEventDestroy(arm_done), "cudaEventDestroy arm_done failed");
    if (mode == "full_parallel") {
        check_cuda(cudaStreamDestroy(vehicle_stream), "cudaStreamDestroy vehicle failed");
        check_cuda(cudaStreamDestroy(arm_stream), "cudaStreamDestroy arm failed");
        check_cuda(cudaStreamDestroy(timer_stream), "cudaStreamDestroy timer failed");
    }
    return static_cast<double>(elapsed_ms) / static_cast<double>(args.iters);
}

void run_size(int batch_size, const Args& args, const std::string& mode) {
    const MatchedCase data = make_case(batch_size);
    const double cpu_step_ms = run_cpu_size(batch_size, args, data, mode);
    const double gpu_step_ms = run_gpu_size(batch_size, args, data, mode);
    const double cpu_env_steps_per_sec =
        static_cast<double>(batch_size) * 1000.0 / std::max(cpu_step_ms, 1e-9);
    const double gpu_env_steps_per_sec =
        static_cast<double>(batch_size) * 1000.0 / std::max(gpu_step_ms, 1e-9);
    const double speedup = cpu_step_ms / std::max(gpu_step_ms, 1e-9);

    std::cout << "mode=" << mode
              << " N=" << batch_size
              << " cpu_step_ms=" << cpu_step_ms
              << " cpu_env_steps_per_sec=" << cpu_env_steps_per_sec
              << " gpu_step_ms=" << gpu_step_ms
              << " gpu_env_steps_per_sec=" << gpu_env_steps_per_sec
              << " gpu_speedup=" << speedup
              << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        int device = 0;
        check_cuda(cudaGetDevice(&device), "cudaGetDevice failed");
        cudaDeviceProp prop{};
        check_cuda(cudaGetDeviceProperties(&prop, device), "cudaGetDeviceProperties failed");
        std::cout << "device=" << device << " name=\"" << prop.name << "\""
                  << " gpu_threads=" << args.threads
                  << " cpu_threads=" << args.cpu_threads
                  << " warmup=" << args.warmup
                  << " iters=" << args.iters
                  << " mode=" << args.mode << '\n';
        const std::vector<std::string> modes =
            args.mode == "all" ? std::vector<std::string>{"vehicle", "arm", "full", "full_parallel"} :
                                 std::vector<std::string>{args.mode};
        for (const std::string& mode : modes) {
            for (const int size : args.sizes) {
                run_size(size, args, mode);
            }
        }
    } catch (const std::exception& ex) {
        std::cerr << "uvms_gpu_dynamics_benchmark: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
