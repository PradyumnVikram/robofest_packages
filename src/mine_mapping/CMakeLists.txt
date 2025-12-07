cmake_minimum_required(VERSION 3.8)
project(mine_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)

# ← ADD EIGEN3
find_package(Eigen3 REQUIRED)

include_directories(src/)

# Library - ADD Eigen3
add_library(map_generator SHARED src/map_generator.cpp)
ament_target_dependencies(map_generator rclcpp sensor_msgs cv_bridge OpenCV px4_msgs)
target_link_libraries(map_generator ${OpenCV_LIBRARIES} Eigen3::Eigen)

# Executable - ADD Eigen3  
add_executable(mine_mapper src/mine_map_node.cpp)
ament_target_dependencies(mine_mapper rclcpp sensor_msgs cv_bridge OpenCV px4_msgs)
target_link_libraries(mine_mapper 
  map_generator 
  ${OpenCV_LIBRARIES} 
  cv_bridge::cv_bridge
  Eigen3::Eigen  # ← ADD THIS
)

# Install unchanged
install(TARGETS mine_mapper map_generator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
