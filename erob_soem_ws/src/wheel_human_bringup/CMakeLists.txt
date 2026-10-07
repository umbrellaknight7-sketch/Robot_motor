cmake_minimum_required(VERSION 3.16)
project(wheel_human_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # These pure model tests need only pytest's built-in fixtures. Avoid loading
  # unrelated ROS pytest plugins from the user's underlay.
  ament_add_pytest_test(model_contract test/test_model_contract.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
      PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()

ament_package()
