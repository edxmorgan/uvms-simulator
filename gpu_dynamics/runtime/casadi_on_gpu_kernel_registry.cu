// GPU kernel registry for the UVMS batch dynamics.
// Author: edward morgan
#include "casadi_on_gpu_kernel_registry.h"
#include "Mnext_reg_fe1_cse.cuh"
#include "Vnext_reg_gpu.cuh"

extern "C" CUDA_GLOBAL void Mnext_reg_fe1x5_gpu_kernel(
    const casadi_real* i0_in,
    const casadi_real* i1_in,
    const casadi_real* i2_in,
    const casadi_real* i3_in,
    const casadi_real* i4_in,
    const casadi_real* i5_in,
    const casadi_real* i6_in,
    const casadi_real* i7_in,
    const casadi_real* i8_in,
    casadi_real* o0_out,
    int n_candidates);

namespace casadi_on_gpu {
namespace {
void launch_Vnext_reg(const std::uintptr_t* input_ptrs,
                      const std::uintptr_t* output_ptrs,
                      int blocks,
                      int threads_per_block,
                      cudaStream_t stream,
                      int n_candidates) {
  auto* i0 = reinterpret_cast<const casadi_real*>(input_ptrs[0]);
  auto* i1 = reinterpret_cast<const casadi_real*>(input_ptrs[1]);
  auto* i2 = reinterpret_cast<const casadi_real*>(input_ptrs[2]);
  auto* i3 = reinterpret_cast<const casadi_real*>(input_ptrs[3]);
  auto* i4 = reinterpret_cast<const casadi_real*>(input_ptrs[4]);
  auto* o0 = reinterpret_cast<casadi_real*>(output_ptrs[0]);
  Vnext_reg_gpu_kernel<<<blocks, threads_per_block, 0, stream>>>(i0, i1, i2, i3, i4, o0, n_candidates);
}

void launch_Mnext_reg(const std::uintptr_t* input_ptrs,
                      const std::uintptr_t* output_ptrs,
                      int blocks,
                      int threads_per_block,
                      cudaStream_t stream,
                      int n_candidates) {
  auto* i0 = reinterpret_cast<const casadi_real*>(input_ptrs[0]);
  auto* i1 = reinterpret_cast<const casadi_real*>(input_ptrs[1]);
  auto* i2 = reinterpret_cast<const casadi_real*>(input_ptrs[2]);
  auto* i3 = reinterpret_cast<const casadi_real*>(input_ptrs[3]);
  auto* i4 = reinterpret_cast<const casadi_real*>(input_ptrs[4]);
  auto* i5 = reinterpret_cast<const casadi_real*>(input_ptrs[5]);
  auto* i6 = reinterpret_cast<const casadi_real*>(input_ptrs[6]);
  auto* i7 = reinterpret_cast<const casadi_real*>(input_ptrs[7]);
  auto* i8 = reinterpret_cast<const casadi_real*>(input_ptrs[8]);
  auto* o0 = reinterpret_cast<casadi_real*>(output_ptrs[0]);
  Mnext_reg_fe1x5_gpu_kernel<<<blocks, threads_per_block, 0, stream>>>(
      i0, i1, i2, i3, i4, i5, i6, i7, i8, o0, n_candidates);
}

const KernelEntry kRegistry[] = {
  {"Vnext_reg",
   "Vnext_reg_gpu_kernel",
   5,
   1,
   5,
   1,
   0,
   830,
   {0, 1, 2, 3, 4},
   {12, 6, 33, 1, 6},
   {12},
   launch_Vnext_reg},
  {"Mnext_reg",
   "Mnext_reg_fe1x5_gpu_kernel",
   9,
   1,
   9,
   1,
   0,
   1028,
   {0, 1, 2, 3, 4, 5, 6, 7, 8},
   {10, 5, 1, 81, 1, 1, 1, 4, 1},
   {10},
   launch_Mnext_reg},
};

}  // namespace

const KernelEntry* get_kernel_registry(std::size_t* count) {
  *count = sizeof(kRegistry) / sizeof(kRegistry[0]);
  return kRegistry;
}

}  // namespace casadi_on_gpu
