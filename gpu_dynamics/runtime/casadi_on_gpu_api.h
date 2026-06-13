#pragma once

// Author: edward morgan

#include <cstdint>
#include <string>
#include <vector>

namespace casadi_on_gpu {

struct KernelMetadata {
    std::string function_name;
    std::string kernel_name;
    std::vector<int> batch_inputs;
    std::vector<int> input_nnz;
    std::vector<int> output_nnz;
};

void launch(const std::string& function_name,
            const std::vector<std::uintptr_t>& input_ptrs,
            const std::vector<std::uintptr_t>& output_ptrs,
            int n_candidates,
            int threads_per_block,
            std::uintptr_t stream_ptr,
            bool sync);

std::vector<KernelMetadata> list_kernels();

void device_synchronize();

}  // namespace casadi_on_gpu
