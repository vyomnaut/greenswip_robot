cmake_minimum_required(VERSION 3.8)
project(greenswip_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package (nodes)
ament_python_install_package(${PROJECT_NAME})

# Install entry-point scripts
install(PROGRAMS
  greenswip_robot/perception_node.py
  greenswip_robot/control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, urdf, worlds, config directories
install(DIRECTORY launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
