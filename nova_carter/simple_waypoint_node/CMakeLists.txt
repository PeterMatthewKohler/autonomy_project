cmake_minimum_required(VERSION 3.8)
project(simple_waypoint_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nova_carter_interfaces REQUIRED)

add_library(WaypointNode_lib STATIC
  src/WaypointNode_impl.cpp
  include/waypoint_node/WaypointNode_impl.hpp
)
target_compile_features(WaypointNode_lib PUBLIC cxx_std_17)
target_include_directories(WaypointNode_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(WaypointNode_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nova_carter_interfaces
)

add_executable(WaypointNode_exe src/WaypointNode_main.cpp)
target_link_libraries(WaypointNode_exe WaypointNode_lib)

install(TARGETS WaypointNode_exe
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(controller test/test_controller.py TIMEOUT 60)
  # Match the existing package until a project license is chosen.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
