// Keep generated device functions and dispatch kernels in one CUDA translation unit.
// This avoids requiring relocatable device-code linking for the packaged ROS build.

#define casadi_s0_h Mnext_reg_fe5_s0_h
#define casadi_s1_h Mnext_reg_fe5_s1_h
#define casadi_s2_h Mnext_reg_fe5_s2_h
#define casadi_s3_h Mnext_reg_fe5_s3_h
#define casadi_s4_h Mnext_reg_fe5_s4_h
#include "Mnext_reg_fe5.cu"
#undef casadi_s0_h
#undef casadi_s1_h
#undef casadi_s2_h
#undef casadi_s3_h
#undef casadi_s4_h
#undef CASADI_PREFIX

#define casadi_s0_h Vnext_reg_s0_h
#define casadi_s1_h Vnext_reg_s1_h
#define casadi_s2_h Vnext_reg_s2_h
#define casadi_s3_h Vnext_reg_s3_h
#define casadi_s4_h Vnext_reg_s4_h
#include "Vnext_reg.cu"
#undef casadi_s0_h
#undef casadi_s1_h
#undef casadi_s2_h
#undef casadi_s3_h
#undef casadi_s4_h
#undef CASADI_PREFIX

#include "casadi_on_gpu_kernel_registry.cu"
