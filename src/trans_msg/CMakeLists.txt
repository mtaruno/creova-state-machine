cmake_minimum_required(VERSION 3.5)
project(trans_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # ✅ ADD THIS LINE

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FinalDetection.msg"
  DEPENDENCIES std_msgs
)

ament_package()
