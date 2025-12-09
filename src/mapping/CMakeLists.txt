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
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include src directory for custom headers
include_directories(src/)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# === INTEGRATED DRONE MAPPING NODE ===
add_executable(integrated_drone_mapping_node
  src/integrated_drone_mapping_node.cpp
)
ament_target_dependencies(integrated_drone_mapping_node
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
)

# === CORRIDOR GENERATION NODE ===
add_executable(corridor_gen_correct 
  src/corridor_gen_correct.cpp
  src/final_hstar.hpp      # Custom header
  src/quickhull2.hpp       # Custom header
  src/linear_eq.hpp        # ← ADDED linear_eq.hpp
)
ament_target_dependencies(corridor_gen_correct
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(corridor_gen_correct 
  Eigen3::Eigen
)

# Compiler optimizations
target_compile_options(integrated_drone_mapping_node PRIVATE
  -O3 -march=native -DNDEBUG
)
target_compile_options(corridor_gen_correct PRIVATE
  -O3 -march=native -DNDEBUG
)

target_compile_options(corridor_gen_correct PRIVATE
  -O3 -march=native -DNDEBUG -Wno-unused-parameter -Wno-sign-compare
)

# Install executables
install(TARGETS
  integrated_drone_mapping_node
  corridor_gen_correct
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory if exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
