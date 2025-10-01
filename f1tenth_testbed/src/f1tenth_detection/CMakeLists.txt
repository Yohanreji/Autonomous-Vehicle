cmake_minimum_required(VERSION 3.5)
project(f1tenth_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)

# Install Python scripts
install(PROGRAMS
  f1tenth_detection/f1tenth_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python dependencies
ament_python_install_package(${PROJECT_NAME})

ament_package()

