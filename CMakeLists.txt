cmake_minimum_required(VERSION 3.5)
project(fuzzy_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Create Cpp executable
add_executable(ts_fuzzy_test1 src/ts_fuzzy_test1.cpp)
add_executable(ts_fuzzy_test2 src/ts_fuzzy_test2.cpp)
add_executable(ts_fuzzy_controller src/ts_fuzzy_node.cpp src/ts_fuzzy_controller_component.cpp)
ament_target_dependencies(ts_fuzzy_test1
    rclcpp
)
ament_target_dependencies(ts_fuzzy_test2
    rclcpp
)
ament_target_dependencies(ts_fuzzy_controller
    rclcpp
    sensor_msgs
)

# Install Cpp executables
install(TARGETS
  ts_fuzzy_test1
  ts_fuzzy_test2
  ts_fuzzy_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()