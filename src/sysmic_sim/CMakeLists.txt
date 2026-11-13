cmake_minimum_required(VERSION 3.8)
project(sysmic_sim)

find_package(ament_cmake REQUIRED)

# Robot description, launch files, RViz config and the simulation scripts.
# The simulation scripts (sim/) run on the conda Python, not as ROS nodes.
install(
  DIRECTORY description launch rviz sim
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# ROS 2 node
install(
  PROGRAMS scripts/ros2_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
