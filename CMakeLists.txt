cmake_minimum_required(VERSION 3.16)
project(ros2_control_blue_reach_5 LANGUAGES C CXX)

# ROS Jazzy dependencies still rely on FindBoost in some exported CMake files.
# Keep the legacy behavior enabled so CMake 3.30+ does not warn during configure.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(UVMS_ENABLE_CUDA_DYNAMICS "AUTO" CACHE STRING "Build CUDA UVMS dynamics: AUTO, ON, or OFF")
set_property(CACHE UVMS_ENABLE_CUDA_DYNAMICS PROPERTY STRINGS AUTO ON OFF)
option(UVMS_BUILD_DOCS "Build local Sphinx HTML docs during colcon build" ON)
option(UVMS_CUDA_FAST_COMPILE "Prefer faster NVCC compile time over maximum generated CUDA kernel runtime optimization" ON)
set(UVMS_CUDA_ARCHITECTURES "" CACHE STRING
  "CUDA architectures for optional UVMS GPU dynamics. Empty means use CMake native detection when available, or honor CMAKE_CUDA_ARCHITECTURES/CUDAARCHS.")
set(UVMS_CUDA_DYNAMICS_AVAILABLE FALSE)

if(NOT UVMS_ENABLE_CUDA_DYNAMICS STREQUAL "OFF")
  if(DEFINED ENV{CUDACXX} AND EXISTS "$ENV{CUDACXX}")
    set(CMAKE_CUDA_COMPILER "$ENV{CUDACXX}" CACHE FILEPATH "CUDA compiler" FORCE)
  endif()
  find_program(UVMS_NVCC_EXECUTABLE NAMES nvcc
    PATHS /usr/local/cuda/bin
    NO_DEFAULT_PATH
  )
  if(NOT UVMS_NVCC_EXECUTABLE)
    find_program(UVMS_NVCC_EXECUTABLE NAMES nvcc)
  endif()
  if(UVMS_NVCC_EXECUTABLE)
    if(NOT DEFINED CMAKE_CUDA_COMPILER OR
       CMAKE_CUDA_COMPILER STREQUAL "" OR
       CMAKE_CUDA_COMPILER STREQUAL "/usr/bin/nvcc" OR
       CMAKE_CUDA_COMPILER STREQUAL "/bin/nvcc")
      set(CMAKE_CUDA_COMPILER "${UVMS_NVCC_EXECUTABLE}" CACHE FILEPATH "CUDA compiler" FORCE)
    endif()
    set(UVMS_CAN_ENABLE_CUDA_DYNAMICS TRUE)
    if(UVMS_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES "${UVMS_CUDA_ARCHITECTURES}" CACHE STRING "CUDA architectures" FORCE)
    elseif(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
      if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
        set(CMAKE_CUDA_ARCHITECTURES "native" CACHE STRING "CUDA architectures" FORCE)
      elseif(UVMS_ENABLE_CUDA_DYNAMICS STREQUAL "ON")
        message(FATAL_ERROR "UVMS_ENABLE_CUDA_DYNAMICS=ON needs an explicit CUDA architecture with CMake < 3.24. Pass -DUVMS_CUDA_ARCHITECTURES=<arch>, for example 120 for RTX 5090.")
      else()
        set(UVMS_CAN_ENABLE_CUDA_DYNAMICS FALSE)
        message(STATUS "UVMS CUDA dynamics disabled: CMake < 3.24 cannot reliably infer the native CUDA architecture. Pass -DUVMS_CUDA_ARCHITECTURES=<arch> to enable it.")
      endif()
    endif()

    if(UVMS_CAN_ENABLE_CUDA_DYNAMICS)
      if(CMAKE_CUDA_ARCHITECTURES MATCHES "^[0-9]+-virtual$")
        message(WARNING "UVMS CUDA dynamics is configured for PTX-only architecture '${CMAKE_CUDA_ARCHITECTURES}'. Prefer a real/native architecture for runtime compatibility, for example -DUVMS_CUDA_ARCHITECTURES=120 on RTX 5090.")
      endif()
      enable_language(CUDA)
      find_package(CUDAToolkit REQUIRED)
      set(UVMS_CUDA_DYNAMICS_AVAILABLE TRUE)
      message(STATUS "UVMS CUDA dynamics enabled with CMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}")
    endif()
  elseif(UVMS_ENABLE_CUDA_DYNAMICS STREQUAL "ON")
    message(FATAL_ERROR "UVMS_ENABLE_CUDA_DYNAMICS=ON but nvcc was not found. Set CUDACXX=/path/to/nvcc or use UVMS_ENABLE_CUDA_DYNAMICS=OFF.")
  else()
    message(STATUS "UVMS CUDA dynamics disabled: nvcc not found")
  endif()
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  mavros_msgs
  rcl_interfaces
  casadi
  eigen3_cmake_module
  Eigen3
  tf2_msgs
  tf2
  tf2_ros
  realtime_tools
  tf2_geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  mavlink
  std_srvs
  angles
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pybind11 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamicObstacle.msg"
  "msg/DynamicObstacleArray.msg"
  "msg/SimCameraDynamics.msg"
  "msg/SimVehicleDynamics.msg"
  "msg/SimManipulatorDynamics.msg"
  "msg/BatchCommand.msg"
  "msg/BatchObservation.msg"
  "srv/ResetBatchSim.srv"
  "srv/ResetSimUvms.srv"
  "srv/SetDynamicObstacles.srv"
  "srv/SetSimCameraDynamics.srv"
  "srv/SetSimDynamics.srv"
  DEPENDENCIES geometry_msgs std_msgs visualization_msgs
)

# --- Begin GStreamer additions ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

message(STATUS "GStreamer include dirs: ${GSTREAMER_INCLUDE_DIRS}")
message(STATUS "GStreamer libraries: ${GSTREAMER_LIBRARIES}")
message(STATUS "GStreamer App include dirs: ${GSTREAMER_APP_INCLUDE_DIRS}")
message(STATUS "GStreamer App libraries: ${GSTREAMER_APP_LIBRARIES}")
# --- End GStreamer additions ---

set(PROJECT_LIB ${PROJECT_NAME}_hardware)
set(BATCH_UVMS_CORE_LIB ${PROJECT_NAME}_batch_uvms_core)

add_subdirectory(cpu_dynamics)
message(STATUS "UVMS build: CPU batch dynamics enabled (target=${PROJECT_NAME}_uvms_cpu_dynamics)")

if(UVMS_CUDA_DYNAMICS_AVAILABLE)
  add_subdirectory(gpu_dynamics)
  message(STATUS "UVMS build: GPU batch dynamics enabled (target=${PROJECT_NAME}_uvms_gpu_dynamics, nvcc=${CMAKE_CUDA_COMPILER}, arch=${CMAKE_CUDA_ARCHITECTURES}, fast_compile=${UVMS_CUDA_FAST_COMPILE})")
else()
  message(STATUS "UVMS build: GPU batch dynamics disabled (UVMS_ENABLE_CUDA_DYNAMICS=${UVMS_ENABLE_CUDA_DYNAMICS})")
endif()

add_library(
  ${BATCH_UVMS_CORE_LIB}
  STATIC
  hardware/batch_uvms_core.cpp
)
target_compile_features(${BATCH_UVMS_CORE_LIB} PUBLIC cxx_std_17)
target_include_directories(${BATCH_UVMS_CORE_LIB} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ros2_control_blue_reach_5>
)
target_link_libraries(${BATCH_UVMS_CORE_LIB} PUBLIC ${PROJECT_NAME}_uvms_cpu_dynamics)
set_target_properties(${BATCH_UVMS_CORE_LIB} PROPERTIES POSITION_INDEPENDENT_CODE ON)
message(STATUS "UVMS build: batch core enabled (target=${BATCH_UVMS_CORE_LIB})")

## COMPILE
add_library(
  ${PROJECT_LIB}
  SHARED
  hardware/batch_sim_uvms_system_hardware.cpp
  hardware/sim_reach_system_multi_interface.cpp
  hardware/reach_system_multi_Interface.cpp
  hardware/utils.cpp
  hardware/cobs.cpp
  hardware/crc.cpp
  hardware/driver.cpp
  hardware/packet.cpp
  hardware/serial_client.cpp
  hardware/sim_vehicle_system_multi_interface.cpp
  hardware/bluerov_system_multi_interface.cpp
  hardware/vehicle.cpp
  hardware/joint.cpp
  hardware/dvldriver.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_LIB} PUBLIC "${cpp_typesupport_target}")

target_link_libraries(${PROJECT_LIB} PUBLIC casadi ${BATCH_UVMS_CORE_LIB})

target_compile_features(${PROJECT_LIB} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_LIB} PRIVATE
  $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang>:-Wno-address-of-packed-member>
)
target_include_directories(${PROJECT_LIB} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ros2_control_blue_reach_5>
)
ament_target_dependencies(
  ${PROJECT_LIB} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
set_target_properties(${PROJECT_LIB} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_LIB} PRIVATE "ROS2_CONTROL_BLUE_REACH_5_BUILDING_DLL")
message(STATUS "UVMS build: ros2_control hardware plugin enabled (target=${PROJECT_LIB})")

pybind11_add_module(_batch_uvms_core hardware/python/batch_uvms_core_pybind.cpp)
target_compile_features(_batch_uvms_core PUBLIC cxx_std_17)
target_link_libraries(_batch_uvms_core PRIVATE ${BATCH_UVMS_CORE_LIB})
target_include_directories(_batch_uvms_core PRIVATE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
)
message(STATUS "UVMS build: Python batch core extension enabled (target=_batch_uvms_core)")

if(UVMS_CUDA_DYNAMICS_AVAILABLE)
  pybind11_add_module(_batch_uvms_gpu_core hardware/python/batch_uvms_gpu_core_pybind.cpp)
  target_compile_features(_batch_uvms_gpu_core PUBLIC cxx_std_17)
  target_link_libraries(_batch_uvms_gpu_core PRIVATE
    ${PROJECT_NAME}_uvms_gpu_dynamics
    CUDA::cudart
  )
  target_include_directories(_batch_uvms_gpu_core PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/gpu_dynamics/include>
  )
  message(STATUS "UVMS build: Python GPU batch core extension enabled (target=_batch_uvms_gpu_core)")
endif()

add_executable(gstreamer_camera_node
  hardware/gstreamer_camera_node.cpp
  hardware/gstreamer_camera_driver.cpp
)
target_compile_features(gstreamer_camera_node PUBLIC cxx_std_17)
target_include_directories(gstreamer_camera_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)
target_link_libraries(gstreamer_camera_node PUBLIC
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)
ament_target_dependencies(gstreamer_camera_node PUBLIC
  rclcpp
  realtime_tools
  sensor_msgs
)
message(STATUS "UVMS build: GStreamer camera node enabled (target=gstreamer_camera_node)")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_blue_reach_5.xml)

# Collect all files within the casadi_lib directory and store them in a variable
file(GLOB CASADI_LIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/casadi_lib/*")

# INSTALL

# Install all files from the collected list to the destination directory
install(FILES ${CASADI_LIB_FILES} DESTINATION lib)

install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY description/launch description/ros2_control description/xacro description/rviz description/alpha description/Bathymetry description/blue
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)


# Install the Python package "bringup"  ← ADDED
ament_python_install_package(bringup)

install(TARGETS ${PROJECT_LIB} ${BATCH_UVMS_CORE_LIB}
  EXPORT export_ros2_control_blue_reach_5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS _batch_uvms_core
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

if(UVMS_CUDA_DYNAMICS_AVAILABLE)
  install(TARGETS _batch_uvms_gpu_core
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  )
endif()

install(TARGETS gstreamer_camera_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  bringup/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  bringup/sim_camera_renderer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sim_camera_renderer_node
)

install(PROGRAMS
  bringup/dynamic_obstacle_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamic_obstacle_sim_node
)

if(UVMS_BUILD_DOCS)
  find_program(UVMS_SPHINX_BUILD_EXECUTABLE NAMES sphinx-build)
  if(NOT UVMS_SPHINX_BUILD_EXECUTABLE)
    message(FATAL_ERROR "UVMS_BUILD_DOCS=ON requires sphinx-build. Install doc dependencies with: python3 -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/doc/requirements-docs.txt")
  endif()

  add_custom_target(uvms_sphinx_html ALL
    COMMAND ${UVMS_SPHINX_BUILD_EXECUTABLE} -M html
            ${CMAKE_CURRENT_SOURCE_DIR}/doc
            ${CMAKE_CURRENT_SOURCE_DIR}/doc/_build
            -E -a
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
    COMMENT "Building UVMS Sphinx HTML documentation"
    VERBATIM
  )
  message(STATUS "UVMS build: local docs enabled (target=uvms_sphinx_html)")
else()
  message(STATUS "UVMS build: local docs disabled (UVMS_BUILD_DOCS=OFF)")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_blue_reach_5 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(rosidl_default_runtime)
if(UVMS_CUDA_DYNAMICS_AVAILABLE)
  ament_export_dependencies(CUDAToolkit)
endif()
ament_package()
