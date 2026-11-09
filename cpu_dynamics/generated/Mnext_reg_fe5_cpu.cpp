#include "ros2_control_blue_reach_5/arm_integration_policy.hpp"
#include <limits>

// Build the generated CasADi arm dynamics with a normal C++ compiler.
// The generated CUDA kernel wrapper is compiled but unused on the CPU path.
namespace {
struct CpuCudaDim3 { int x; };
CpuCudaDim3 blockIdx{0};
CpuCudaDim3 blockDim{1};
CpuCudaDim3 threadIdx{0};
}  // namespace

// Preserve the generated five-substep block; compose six blocks to match
// the main arm plant's 30 RK4 substeps without unrolling a huge expression.
#define Mnext_reg Mnext_reg_fe5_internal
#include "Mnext_reg_fe5.cu"
#undef Mnext_reg

extern "C" int Mnext_reg(const double** arg, double** res,
                         long long* iw, double* w, int mem) {
  const int count = ros2_control_blue_reach_5::arm_integration_policy::count_or_error(arg[2][0], arg[8][0]);
  if (count < 0) {
    for (int i = 0; i < 10; ++i) res[0][i] = std::numeric_limits<double>::quiet_NaN();
    return count;
  }
  double state[10];
  double next[10];
  for (int i = 0; i < 10; ++i) state[i] = arg[0][i];
  const double dt = (arg[2][0] / count) / 6.0;
  const double* inputs[9];
  for (int i = 0; i < 9; ++i) inputs[i] = arg[i];
  inputs[0] = state;
  inputs[2] = &dt;
  double* outputs[1] = {next};
  for (int step = 0; step < 6 * count; ++step) {
    const int status = Mnext_reg_fe5_internal(inputs, outputs, iw, w, mem);
    if (status) return status;
    for (int i = 0; i < 10; ++i) state[i] = next[i];
  }
  for (int i = 0; i < 10; ++i) res[0][i] = state[i];
  return 0;
}
