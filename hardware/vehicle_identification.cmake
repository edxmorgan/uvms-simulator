# Explicit opt-in; ordinary simulation requires neither SDK nor MOSEK license.
option(UVMS_BUILD_VEHICLE_IDENTIFIER "Build native MOSEK vehicle identifier" OFF)
set(UVMS_MOSEK_PLATFORM "" CACHE PATH "MOSEK platform directory containing h, bin, src/fusion_cxx")
if(UVMS_BUILD_VEHICLE_IDENTIFIER)
  find_path(UVMS_MOSEK_INCLUDE fusion.h HINTS "${UVMS_MOSEK_PLATFORM}/h" REQUIRED NO_DEFAULT_PATH)
  find_library(UVMS_MOSEK_LIBRARY mosek64 HINTS "${UVMS_MOSEK_PLATFORM}/bin" REQUIRED NO_DEFAULT_PATH)
  set(_fusion_sources)
  foreach(_source fusion BaseModel Debug mosektask IntMap SolverInfo StringBuffer)
    list(APPEND _fusion_sources "${UVMS_MOSEK_PLATFORM}/src/fusion_cxx/${_source}.cc")
  endforeach()
  add_library(uvms_mosek_fusion STATIC ${_fusion_sources})
  target_include_directories(uvms_mosek_fusion SYSTEM PUBLIC "${UVMS_MOSEK_INCLUDE}")
  target_include_directories(uvms_mosek_fusion PRIVATE "${UVMS_MOSEK_PLATFORM}/src/fusion_cxx")
  # SDK implementation warnings are external to this package.
  target_compile_options(uvms_mosek_fusion PRIVATE -w)
  target_link_libraries(uvms_mosek_fusion PUBLIC "${UVMS_MOSEK_LIBRARY}")
  target_compile_features(uvms_mosek_fusion PUBLIC cxx_std_17)
  set_target_properties(uvms_mosek_fusion PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_library(uvms_vehicle_identifier STATIC hardware/vehicle_identifier.cpp
    hardware/vehicle_identification_retention.cpp
    hardware/vehicle_identification_window.cpp
    hardware/generated/vehicle_identification/vehicle_regressor.c)
  target_include_directories(uvms_vehicle_identifier PUBLIC hardware/include
    PRIVATE hardware/generated/vehicle_identification)
  target_link_libraries(uvms_vehicle_identifier PUBLIC Eigen3::Eigen PRIVATE uvms_mosek_fusion)
  target_compile_features(uvms_vehicle_identifier PUBLIC cxx_std_17)
  target_compile_options(uvms_vehicle_identifier PRIVATE -O2)
  find_package(control_msgs REQUIRED)
  add_executable(vehicle_identification_node hardware/vehicle_identification_node.cpp)
  target_link_libraries(vehicle_identification_node PRIVATE uvms_vehicle_identifier "${cpp_typesupport_target}")
  ament_target_dependencies(vehicle_identification_node PUBLIC rclcpp control_msgs std_srvs)
  set_target_properties(vehicle_identification_node PROPERTIES INSTALL_RPATH "${UVMS_MOSEK_PLATFORM}/bin")
  install(TARGETS vehicle_identification_node DESTINATION lib/${PROJECT_NAME})
endif()
