cmake_minimum_required(VERSION 3.11.0 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(LightSensorSystem)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-sensors6 REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})

# Fetch the custom sensor example from ign-sensors
# Users won't commonly use this to fetch their sensors. The sensor may be part
# of the system's CMake project, or installed from another project, etc...
#include(FetchContent)
#FetchContent_Declare(
#  sensors_clone
#  GIT_REPOSITORY https://github.com/ignitionrobotics/ign-sensors
#  GIT_TAG ign-sensors6
#)
#FetchContent_Populate(sensors_clone)
#add_subdirectory(${sensors_clone_SOURCE_DIR}/examples/custom_sensor ${sensors_clone_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  PRIVATE ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../light_sensor/build/liblight_sensor.so
)
target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../light_sensor)

