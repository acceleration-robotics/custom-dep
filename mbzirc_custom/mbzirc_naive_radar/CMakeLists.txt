cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_naive_radar)

find_package(ament_cmake REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_GAZEBO_VER ${gz-sim8_VERSION_MAJOR})
find_package(radar_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclcpp REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# build the GZ-gazebo system
add_library(NaiveRadar SHARED
    src/NaiveRadar.cc
)
target_link_libraries(NaiveRadar PUBLIC
  gz-sim${GZ_GAZEBO_VER}::core
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)

# build the bridge process
add_executable(naive_radar_bridge src/naive_radar_bridge.cc)
ament_target_dependencies(naive_radar_bridge
  PUBLIC
  rclcpp
  radar_msgs
  ros_gz_bridge
)
target_link_libraries(naive_radar_bridge PUBLIC
  gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
  gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
)

# Uncomment the install calls below to install the example naive radar model
install(
  TARGETS NaiveRadar
   DESTINATION lib)
install(DIRECTORY
  models
  launch
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  naive_radar_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
