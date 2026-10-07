cmake_minimum_required(VERSION 3.16)
project(erob_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mock_demo.py scripts/resume_control.py DESTINATION lib/${PROJECT_NAME})

ament_package()
