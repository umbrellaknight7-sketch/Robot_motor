cmake_minimum_required(VERSION 3.16)
project(wheel_human_wrist_adapter)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/wrist_adapter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(wrist_contract test/test_wrist.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/../../../linkerhand-l30-sdk/linker_hand_l30_v6
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
