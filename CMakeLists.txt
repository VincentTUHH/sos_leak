cmake_minimum_required(VERSION 3.5)
project(sos_leak)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


install(PROGRAMS
  nodes/leak_detector.py
  nodes/leak_safety.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
