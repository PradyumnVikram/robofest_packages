cmake_minimum_required(VERSION 3.8)
project(object_tracking_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(swarm_control REQUIRED)  # custom msg package

# Mother node
add_executable(mothernode
  src/mothernode.cpp
)
ament_target_dependencies(mothernode
  rclcpp
  std_msgs
  px4_msgs
  swarm_control
)

# Follower node (executable name: follower_1)
add_executable(follower_1
  src/follower1.cpp
)
ament_target_dependencies(follower_1
  rclcpp
  std_msgs
  px4_msgs
  swarm_control
)

# Install the executables so `ros2 run` can find them
install(TARGETS
  mothernode
  follower_1
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
