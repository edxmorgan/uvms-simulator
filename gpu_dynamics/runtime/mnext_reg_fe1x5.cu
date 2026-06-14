// Author: edward morgan

// Five one-step arm dynamics evaluations. This keeps the GPU build practical
// while preserving the public Mnext_reg signature and lock-mask semantics.

#define casadi_s0_h Mnext_reg_fe1_cse_s0_h
#define casadi_s1_h Mnext_reg_fe1_cse_s1_h
#define casadi_s2_h Mnext_reg_fe1_cse_s2_h
#define casadi_s3_h Mnext_reg_fe1_cse_s3_h
#define casadi_s4_h Mnext_reg_fe1_cse_s4_h
#include "Mnext_reg_fe1_cse.cu"
#undef casadi_s0_h
#undef casadi_s1_h
#undef casadi_s2_h
#undef casadi_s3_h
#undef casadi_s4_h
#undef casadi_f0
#undef casadi_fabs
#undef casadi_s0
#undef casadi_s1
#undef casadi_s2
#undef casadi_s3
#undef casadi_s4
#undef casadi_sign
#undef casadi_sq
#undef CASADI_PREFIX

namespace {

CUDA_DEV void copy10(const casadi_real* src, casadi_real* dst) {
  #pragma unroll
  for (int i = 0; i < 10; ++i) {
    dst[i] = src[i];
  }
}

}  // namespace

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
    int n_candidates) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= n_candidates) {
    return;
  }

  const casadi_real* x_in = i0_in + 10 * idx;
  const casadi_real* tau = i1_in + 5 * idx;
  const casadi_real* arm_params = i3_in + 81 * idx;
  const casadi_real* ee_mass = i4_in + idx;
  const casadi_real* ee_damping = i5_in + idx;
  const casadi_real* ee_stiffness = i6_in + idx;
  const casadi_real* lock_mask = i7_in + 4 * idx;
  const casadi_real* baumgarte_alpha = i8_in + idx;
  casadi_real* x_out = o0_out + 10 * idx;

  casadi_real x_a[10];
  casadi_real x_b[10];
  casadi_real dt_sub[1] = {i2_in[idx] / 5.0F};
  copy10(x_in, x_a);

  for (int step = 0; step < 5; ++step) {
    device_Mnext_reg_fe1_cse_eval(
        x_a, tau, dt_sub, arm_params, ee_mass, ee_damping, ee_stiffness,
        lock_mask, baumgarte_alpha, x_b);
    copy10(x_b, x_a);
  }

  copy10(x_a, x_out);
}
