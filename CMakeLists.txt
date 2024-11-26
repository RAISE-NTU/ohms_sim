cmake_minimum_required(VERSION 3.5)

project(hmrs_sim)

find_package(ament_cmake REQUIRED)

# Updates for comms_emulator_helper
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-common4 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})

add_library(CommsEmulatorHelper
  SHARED
  src/CommsEmulatorHelper.cc
)

target_include_directories(
  CommsEmulatorHelper PRIVATE include
)

target_link_libraries(CommsEmulatorHelper PRIVATE
  ignition-gazebo${GZ_SIM_VER}::ignition-gazebo${GZ_SIM_VER})

install(TARGETS
  CommsEmulatorHelper
  DESTINATION lib/${PROJECT_NAME}
)

# ROS2 build
install(DIRECTORY
  config
  models
  worlds
  launch
  DESTINATION share/${PROJECT_NAME}
)

# setup the environmental hooks for the Gazebo resource path
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
