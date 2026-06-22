// Author: edward morgan

#include "casadi_on_gpu_api.h"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

#include <cuda_runtime.h>

#include "casadi_on_gpu_kernel_registry.h"

namespace casadi_on_gpu {
namespace {
constexpr int kHeapWorkspaceThresholdSlots = 65536;

void throw_on_cuda_error(cudaError_t err, const char* context) {
    if (err == cudaSuccess) {
        return;
    }
    std::string message = context;
    message += ": ";
    message += cudaGetErrorString(err);
    throw std::runtime_error(message);
}

cudaStream_t stream_from_ptr(std::uintptr_t stream_ptr) {
    return reinterpret_cast<cudaStream_t>(stream_ptr);
}

const KernelEntry& find_kernel(const std::string& function_name) {
    std::size_t n = 0;
    const KernelEntry* entries = get_kernel_registry(&n);
    for (std::size_t i = 0; i < n; ++i) {
        if (function_name == entries[i].function_name) {
            return entries[i];
        }
    }
    throw std::invalid_argument("Unknown kernel function: " + function_name);
}

std::size_t estimate_required_stack_bytes(const KernelEntry& entry) {
    const std::size_t ptr_size = sizeof(void*);
    const std::size_t casadi_int_size = sizeof(long long int);
    // Generated units in this repo use casadi_real=float.
    const std::size_t casadi_real_size = sizeof(float);

    const std::size_t arg_slots =
        static_cast<std::size_t>(std::max(1, std::max(entry.sz_arg, entry.n_in)));
    const std::size_t res_slots =
        static_cast<std::size_t>(std::max(1, std::max(entry.sz_res, entry.n_out)));
    const std::size_t iw_slots = static_cast<std::size_t>(std::max(1, entry.sz_iw));
    // Large-workspace kernels are launched with external workspace buffers in the
    // registry path, so they should not inflate stack-limit requests.
    const bool uses_external_workspace = entry.sz_w >= kHeapWorkspaceThresholdSlots;
    const std::size_t w_slots = static_cast<std::size_t>(
        uses_external_workspace ? 1 : std::max(1, entry.sz_w));

    // Keep a safety margin for call-frame overhead and compiler-generated temporaries.
    const std::size_t overhead_bytes = 4u * 1024u;
    return arg_slots * ptr_size + res_slots * ptr_size +
           iw_slots * casadi_int_size + w_slots * casadi_real_size +
           overhead_bytes;
}

void ensure_stack_limit_for_kernel(const KernelEntry& entry) {
    std::size_t current_stack = 0;
    throw_on_cuda_error(
        cudaDeviceGetLimit(&current_stack, cudaLimitStackSize),
        "cudaDeviceGetLimit(cudaLimitStackSize) failed");

    const std::size_t required_stack = estimate_required_stack_bytes(entry);
    if (required_stack > current_stack) {
        throw_on_cuda_error(
            cudaDeviceSetLimit(cudaLimitStackSize, required_stack),
            "cudaDeviceSetLimit(cudaLimitStackSize) failed");
    }
}

}  // namespace

void launch(const std::string& function_name,
            const std::vector<std::uintptr_t>& input_ptrs,
            const std::vector<std::uintptr_t>& output_ptrs,
            int n_candidates,
            int threads_per_block,
            std::uintptr_t stream_ptr,
            bool sync) {
    if (n_candidates <= 0) {
        throw std::invalid_argument("n_candidates must be > 0");
    }
    if (threads_per_block <= 0) {
        throw std::invalid_argument("threads_per_block must be > 0");
    }

    const KernelEntry& entry = find_kernel(function_name);
    ensure_stack_limit_for_kernel(entry);
    if (static_cast<int>(input_ptrs.size()) != entry.n_in) {
        throw std::invalid_argument(
            "input_ptrs size mismatch for " + function_name +
            " (got " + std::to_string(input_ptrs.size()) +
            ", expected " + std::to_string(entry.n_in) + ")");
    }
    if (static_cast<int>(output_ptrs.size()) != entry.n_out) {
        throw std::invalid_argument(
            "output_ptrs size mismatch for " + function_name +
            " (got " + std::to_string(output_ptrs.size()) +
            ", expected " + std::to_string(entry.n_out) + ")");
    }

    cudaStream_t stream = stream_from_ptr(stream_ptr);

    // Some generated kernels have large per-thread local workspace and can fail
    // at launch with cudaErrorMemoryAllocation for larger block sizes. Back off
    // block size automatically before surfacing the error.
    int launch_threads = threads_per_block;
    while (true) {
        const int blocks = (n_candidates + launch_threads - 1) / launch_threads;
        entry.launch(
            input_ptrs.data(),
            output_ptrs.data(),
            blocks,
            launch_threads,
            stream,
            n_candidates);

        const cudaError_t launch_err = cudaGetLastError();
        if (launch_err == cudaSuccess) {
            break;
        }
        if (launch_err == cudaErrorMemoryAllocation && launch_threads > 1) {
            launch_threads = std::max(1, launch_threads / 2);
            continue;
        }
        throw_on_cuda_error(launch_err, "kernel launch failed");
    }

    if (sync) {
        throw_on_cuda_error(cudaStreamSynchronize(stream), "kernel sync failed");
    }
}

std::vector<KernelMetadata> list_kernels() {
    std::size_t n = 0;
    const KernelEntry* entries = get_kernel_registry(&n);

    std::vector<KernelMetadata> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        KernelMetadata m;
        m.function_name = entries[i].function_name;
        m.kernel_name = entries[i].kernel_name;
        m.batch_inputs = entries[i].batch_inputs;
        m.input_nnz = entries[i].input_nnz;
        m.output_nnz = entries[i].output_nnz;
        out.push_back(std::move(m));
    }

    return out;
}

void device_synchronize() {
    throw_on_cuda_error(cudaDeviceSynchronize(), "cudaDeviceSynchronize failed");
}

}  // namespace casadi_on_gpu
