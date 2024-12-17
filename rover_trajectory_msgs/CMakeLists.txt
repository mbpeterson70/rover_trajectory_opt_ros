cmake_minimum_required(VERSION 3.8)
project(rover_trajectory_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverState.msg"
  DEPENDENCIES std_msgs
)

# Install the package
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
