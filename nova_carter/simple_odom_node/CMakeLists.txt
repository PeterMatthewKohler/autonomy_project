cmake_minimum_required(VERSION 3.8)
project(simple_odom_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(OdomNode_SRC
  src/OdomNode_impl.cpp
)

set(OdomNode_HEADERS
  include/odom_node/OdomNode_impl.hpp
)

ament_auto_add_library(OdomNode_lib
  ${OdomNode_SRC}
  ${OdomNode_HEADERS}
)

ament_target_dependencies(OdomNode_lib
    rclcpp
    std_msgs
    nav_msgs
)

ament_auto_add_executable(OdomNode_exe
  src/OdomNode_main.cpp
)

target_link_libraries(OdomNode_exe
  OdomNode_lib
)

# For when you have them...
# # Install parameter files
# install(DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS OdomNode_exe
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
