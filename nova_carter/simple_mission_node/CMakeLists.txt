cmake_minimum_required(VERSION 3.8)
project(simple_mission_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nova_carter_interfaces REQUIRED)

add_library(MissionNode_lib STATIC
  src/MissionNode_impl.cpp
  include/mission_node/MissionNode_impl.hpp
)
target_compile_features(MissionNode_lib PUBLIC cxx_std_17)
target_include_directories(MissionNode_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(MissionNode_lib
  rclcpp
  nav_msgs
  geometry_msgs
  nova_carter_interfaces
)

add_executable(MissionNode_exe src/MissionNode_main.cpp)
target_link_libraries(MissionNode_exe MissionNode_lib)

install(TARGETS MissionNode_exe
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Match the existing packages until a project license is chosen.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
