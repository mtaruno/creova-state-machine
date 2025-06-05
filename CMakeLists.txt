cmake_minimum_required(VERSION 3.8)
project(creova_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Status.msg"
  "msg/PickCommand.msg"
  "srv/GetObjectPose.srv"
  "srv/NavigateToUser.srv"
  "srv/PickObject.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  creova_state_machine/nodes/orchestration_node.py
  creova_state_machine/nodes/manipulation_node.py
  creova_state_machine/nodes/navigation_node.py
  creova_state_machine/nodes/perception_node.py
  creova_state_machine/nodes/physical_ai_node.py
  creova_state_machine/nodes/destination_server_node.py
  creova_state_machine/nodes/get_destination_client.py
  creova_state_machine/nodes/hw_sw_node.py
  creova_state_machine/nodes/test_destination_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other directories
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
