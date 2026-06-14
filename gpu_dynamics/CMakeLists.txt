find_package(CUDAToolkit REQUIRED)

set(UVMS_GPU_DYNAMICS_RUNTIME ${PROJECT_NAME}_uvms_gpu_dynamics_runtime)
set(UVMS_GPU_DYNAMICS_LIB ${PROJECT_NAME}_uvms_gpu_dynamics)

add_library(${UVMS_GPU_DYNAMICS_RUNTIME} OBJECT
  runtime/casadi_on_gpu_kernels.cpp
  runtime/casadi_on_gpu_kernel_registry.cu
  runtime/mnext_reg_fe1x5.cu
  generated/Vnext_reg_gpu.cu
)

# CasADi-generated CUDA kernels are large enough that native Blackwell -O3
# compilation can take impractically long. Keep normal C++ code in Release
# while asking recent NVCC versions to prioritize compile speed for generated
# dynamics. Gate these flags so older CUDA toolkits do not fail on unknown
# options.
set(UVMS_GENERATED_CUDA_COMPILE_OPTIONS
  "-diag-suppress=177"
  "-Xcompiler=-Wno-unused-parameter"
)
if(UVMS_CUDA_FAST_COMPILE AND CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL "13.0")
  list(APPEND UVMS_GENERATED_CUDA_COMPILE_OPTIONS "-Ofc=max" "--split-compile=8")
endif()
set_source_files_properties(
  generated/Vnext_reg_gpu.cu
  runtime/mnext_reg_fe1x5.cu
  PROPERTIES COMPILE_OPTIONS "${UVMS_GENERATED_CUDA_COMPILE_OPTIONS}"
)

set_target_properties(${UVMS_GPU_DYNAMICS_RUNTIME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(${UVMS_GPU_DYNAMICS_RUNTIME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/runtime
  ${CMAKE_CURRENT_SOURCE_DIR}/generated
)

target_compile_options(${UVMS_GPU_DYNAMICS_RUNTIME} PRIVATE
  $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Debug>>:-O0>
  $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:RelWithDebInfo>>:-O1>
)

add_library(${UVMS_GPU_DYNAMICS_LIB} SHARED
  src/uvms_gpu_dynamics.cpp
  src/uvms_gpu_arm_postprocess.cu
  $<TARGET_OBJECTS:${UVMS_GPU_DYNAMICS_RUNTIME}>
)

set_target_properties(${UVMS_GPU_DYNAMICS_LIB} PROPERTIES
  OUTPUT_NAME uvms_gpu_dynamics
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CUDA
)

target_compile_features(${UVMS_GPU_DYNAMICS_LIB} PUBLIC cxx_std_17)
target_include_directories(${UVMS_GPU_DYNAMICS_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/runtime>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${UVMS_GPU_DYNAMICS_LIB} PUBLIC CUDA::cudart)
target_link_options(${UVMS_GPU_DYNAMICS_LIB} PRIVATE -fno-lto)

add_executable(uvms_gpu_dynamics_benchmark
  benchmark/uvms_gpu_dynamics_benchmark.cpp
)
target_compile_features(uvms_gpu_dynamics_benchmark PRIVATE cxx_std_17)
target_include_directories(uvms_gpu_dynamics_benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${PROJECT_SOURCE_DIR}/cpu_dynamics/include
)
target_link_libraries(uvms_gpu_dynamics_benchmark PRIVATE
  ${UVMS_GPU_DYNAMICS_LIB}
  ${PROJECT_NAME}_uvms_cpu_dynamics
  CUDA::cudart
)

add_executable(uvms_gpu_dynamics_correctness
  test/uvms_gpu_dynamics_correctness.cpp
)
target_compile_features(uvms_gpu_dynamics_correctness PRIVATE cxx_std_17)
target_include_directories(uvms_gpu_dynamics_correctness PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/test
  ${PROJECT_SOURCE_DIR}/cpu_dynamics/include
)
target_link_libraries(uvms_gpu_dynamics_correctness PRIVATE
  ${UVMS_GPU_DYNAMICS_LIB}
  ${PROJECT_NAME}_uvms_cpu_dynamics
  CUDA::cudart
)

install(TARGETS ${UVMS_GPU_DYNAMICS_LIB}
  EXPORT export_ros2_control_blue_reach_5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS uvms_gpu_dynamics_benchmark
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS uvms_gpu_dynamics_correctness
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES generated/kernels_manifest.json
  DESTINATION share/${PROJECT_NAME}/gpu_dynamics
)
