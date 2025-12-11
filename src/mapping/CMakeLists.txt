cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core dependencies - OMPL added here
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED) 
find_package(geometry_msgs REQUIRED) # Added for GCOPTER

# All your custom headers live in src/
include_directories(src/)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

##################################################
# 1) Integrated drone mapping node
##################################################
add_executable(integrated_drone_mapping_node
  src/integrated_drone_mapping_node.cpp
)
ament_target_dependencies(integrated_drone_mapping_node
  rclcpp px4_msgs sensor_msgs nav_msgs
)
target_compile_options(integrated_drone_mapping_node PRIVATE -O3 -march=native -DNDEBUG)

##################################################
# 2) Corridor generation node
##################################################
add_executable(corridor_gen_correct 
  src/corridor_gen_correct.cpp
)
ament_target_dependencies(corridor_gen_correct
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs Eigen3
)
target_link_libraries(corridor_gen_correct Eigen3::Eigen)
target_compile_options(corridor_gen_correct PRIVATE
  -O3 -march=native -DNDEBUG -Wno-unused-parameter -Wno-sign-compare
)

##################################################
# 3) GCOPTER trajectory planner node - OMPL deps added
##################################################
add_executable(gcopter_trajectory_planner src/planner.cpp)

ament_target_dependencies(gcopter_trajectory_planner
  rclcpp std_msgs Eigen3 ompl px4_msgs# Added ompl
)

target_link_libraries(gcopter_trajectory_planner
  Eigen3::Eigen
  ompl::ompl  # Added OMPL library
)

target_include_directories(gcopter_trajectory_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${ompl_INCLUDE_DIRS}  # Added OMPL includes (symlink covers this)
)

target_compile_options(gcopter_trajectory_planner PRIVATE
  -O3 -march=native -DNDEBUG -Wno-unused-parameter -Wno-sign-compare
)

##################################################
# Install
##################################################
install(TARGETS
  integrated_drone_mapping_node
  corridor_gen_correct
  gcopter_trajectory_planner
  DESTINATION lib/${PROJECT_NAME}
)

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
