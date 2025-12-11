cmake_minimum_required(VERSION 3.8)
project(swarm_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add std_msgs or others only if used in your .msg
# find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotherState.msg"
  # DEPENDENCIES std_msgs    # uncomment only if using std_msgs types
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
