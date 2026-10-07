cmake_minimum_required(VERSION 3.16)
project(wheel_human_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/cad_joint_state_publisher_gui.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf meshes rviz launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cad_model test/test_cad_model.py)
endif()

ament_package()
