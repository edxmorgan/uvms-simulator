set(UVMS_CPU_DYNAMICS_LIB ${PROJECT_NAME}_uvms_cpu_dynamics)

add_library(${UVMS_CPU_DYNAMICS_LIB} SHARED
  generated/Mnext_reg_fe5_cpu.cpp
  generated/Vnext_reg_cpu.cpp
  src/uvms_cpu_dynamics.cpp
)

target_compile_features(${UVMS_CPU_DYNAMICS_LIB} PUBLIC cxx_std_17)
target_include_directories(${UVMS_CPU_DYNAMICS_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${UVMS_CPU_DYNAMICS_LIB} PRIVATE
  ${PROJECT_SOURCE_DIR}/gpu_dynamics/generated
)
set_target_properties(${UVMS_CPU_DYNAMICS_LIB} PROPERTIES
  OUTPUT_NAME uvms_cpu_dynamics
  POSITION_INDEPENDENT_CODE ON
)

target_compile_options(${UVMS_CPU_DYNAMICS_LIB} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-unused-parameter>
)

pybind11_add_module(_uvms_cpu_dynamics src/uvms_cpu_dynamics_pybind.cpp)
target_compile_features(_uvms_cpu_dynamics PUBLIC cxx_std_17)
target_link_libraries(_uvms_cpu_dynamics PRIVATE ${UVMS_CPU_DYNAMICS_LIB})
target_include_directories(_uvms_cpu_dynamics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS ${UVMS_CPU_DYNAMICS_LIB}
  EXPORT export_ros2_control_blue_reach_5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS _uvms_cpu_dynamics
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
