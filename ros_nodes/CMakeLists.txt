cmake_minimum_required(VERSION 3.0.2)
project(car_nodes)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  can_msgs
)

## Install Python scripts
install(PROGRAMS
  scripts/can_bridge.py
  scripts/can_engine_node.py
  scripts/transmission_control_node.py
  scripts/brake_system_node.py
  scripts/steering_system_node.py
  scripts/vehicle_speed_node.py
  scripts/car_monitor_node.py
  scripts/Dashboard/app.py
  scripts/Dashboard/dashboard_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs can_msgs
)
