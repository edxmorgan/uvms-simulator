#pragma once

// Author: edward morgan

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <cuda_runtime.h>

namespace casadi_on_gpu {

using KernelLaunchFn = void (*)(const std::uintptr_t* input_ptrs,
                                const std::uintptr_t* output_ptrs,
                                int blocks,
                                int threads_per_block,
                                cudaStream_t stream,
                                int n_candidates);

struct KernelEntry {
    const char* function_name;
    const char* kernel_name;
    int n_in;
    int n_out;
    int sz_arg;
    int sz_res;
    int sz_iw;
    int sz_w;
    std::vector<int> batch_inputs;
    std::vector<int> input_nnz;
    std::vector<int> output_nnz;
    KernelLaunchFn launch;
};

const KernelEntry* get_kernel_registry(std::size_t* count);

}  // namespace casadi_on_gpu
