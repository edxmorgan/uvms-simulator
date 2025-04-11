cmake_minimum_required(VERSION 3.16)
project(ros2_control_blue_reach_5 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
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
  realtime_tools
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  mavlink
  cv_bridge
  angles
)


# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# --- Begin GStreamer additions ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

message(STATUS "GStreamer include dirs: ${GSTREAMER_INCLUDE_DIRS}")
message(STATUS "GStreamer libraries: ${GSTREAMER_LIBRARIES}")
message(STATUS "GStreamer App include dirs: ${GSTREAMER_APP_INCLUDE_DIRS}")
message(STATUS "GStreamer App libraries: ${GSTREAMER_APP_LIBRARIES}")
# --- End GStreamer additions ---

## COMPILE
add_library(
  ros2_control_blue_reach_5
  SHARED
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

target_link_libraries(ros2_control_blue_reach_5 PUBLIC casadi)
# Link against GStreamer libraries as well.
target_link_libraries(ros2_control_blue_reach_5 PUBLIC
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  ${cv_bridge_LIBRARIES}  # Link with cv_bridge libraries.
)

target_compile_features(ros2_control_blue_reach_5 PUBLIC cxx_std_17)
target_include_directories(ros2_control_blue_reach_5 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ros2_control_blue_reach_5>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)
ament_target_dependencies(
  ros2_control_blue_reach_5 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_BLUE_REACH_5_BUILDING_DLL")

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
install(TARGETS ${PROJECT_NAME}
  EXPORT export_ros2_control_blue_reach_5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_blue_reach_5 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
