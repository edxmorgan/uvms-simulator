find_package(CUDAToolkit REQUIRED)

set(UVMS_GPU_DYNAMICS_RUNTIME ${PROJECT_NAME}_uvms_gpu_dynamics_runtime)
set(UVMS_GPU_DYNAMICS_LIB ${PROJECT_NAME}_uvms_gpu_dynamics)

add_library(${UVMS_GPU_DYNAMICS_RUNTIME} OBJECT
  runtime/casadi_on_gpu_kernels.cpp
  runtime/uvms_gpu_dynamics_kernels.cu
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

install(TARGETS ${UVMS_GPU_DYNAMICS_LIB}
  EXPORT export_ros2_control_blue_reach_5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES generated/kernels_manifest.json
  DESTINATION share/${PROJECT_NAME}/gpu_dynamics
)
