// Build the generated CasADi arm dynamics with a normal C++ compiler.
// The generated CUDA kernel wrapper is compiled but unused on the CPU path.
namespace {
struct CpuCudaDim3 { int x; };
CpuCudaDim3 blockIdx{0};
CpuCudaDim3 blockDim{1};
CpuCudaDim3 threadIdx{0};
}  // namespace

#include "Mnext_reg_fe5.cu"
