cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add executable
add_executable(integrated_drone_mapping_node
  src/integrated_drone_mapping_node.cpp
)

# Link dependencies
ament_target_dependencies(integrated_drone_mapping_node
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
)

# Compiler optimizations for performance
target_compile_options(integrated_drone_mapping_node PRIVATE
  -O3
  -march=native
  -DNDEBUG
)

# Install executable
install(TARGETS
  integrated_drone_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# ✅ FIXED: Only install launch if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
