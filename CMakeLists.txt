cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME takeoff_plugins)
project(${PROJECT_NAME} VERSION 0.2.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  pluginlib
  takeoff_plugin_base
  rclcpp
  rclcpp_action
  nav_msgs
  std_srvs
  as2_core
  as2_msgs
  mbzirc_msgs
  geometry_msgs
  motion_reference_handlers
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} src/takeoff_plugin_speed.cpp src/takeoff_plugin_position.cpp src/takeoff_plugin_platform.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(takeoff_plugin_base plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
