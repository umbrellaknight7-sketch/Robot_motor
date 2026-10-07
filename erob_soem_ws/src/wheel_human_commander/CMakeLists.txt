cmake_minimum_required(VERSION 3.16)
project(wheel_human_commander)

find_package(ament_cmake REQUIRED)
find_package(pinocchio 3.8.0 EXACT REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
set(dependencies rclcpp rclcpp_action wheel_human_interfaces control_msgs sensor_msgs geometry_msgs trajectory_msgs diagnostic_msgs std_msgs std_srvs erob_driver)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(commander_core src/ownership_manager.cpp src/robot_state_cache.cpp src/goal_validator.cpp src/execution_client.cpp
  src/motion/gravity_compensation.cpp src/motion/trajectory_generator.cpp)
target_compile_features(commander_core PUBLIC cxx_std_17)
target_compile_options(commander_core PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(commander_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(commander_core ${dependencies})

add_executable(commander_node src/commander_node.cpp)
target_link_libraries(commander_node commander_core)

add_library(wheel_human_kinematics SHARED src/motion/kinematics_model.cpp
  src/motion/forward_kinematics.cpp src/motion/inverse_kinematics.cpp)
target_compile_features(wheel_human_kinematics PUBLIC cxx_std_17)
target_compile_options(wheel_human_kinematics PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(wheel_human_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(wheel_human_kinematics PRIVATE pinocchio::pinocchio Eigen3::Eigen)
ament_target_dependencies(wheel_human_kinematics PUBLIC geometry_msgs sensor_msgs)
set_target_properties(wheel_human_kinematics PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(kinematics_node src/kinematics_node.cpp)
target_link_libraries(kinematics_node wheel_human_kinematics)
ament_target_dependencies(kinematics_node rclcpp wheel_human_interfaces)

install(TARGETS commander_core commander_node EXPORT commanderTargets ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS wheel_human_kinematics kinematics_node EXPORT kinematicsTargets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/kinematics_example.py scripts/reference_pose_example.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(commanderTargets HAS_LIBRARY_TARGET)
ament_export_targets(kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_commander test/test_commander.cpp)
  target_link_libraries(test_commander commander_core)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics wheel_human_kinematics Eigen3::Eigen)
  if(TARGET gtest)
    get_target_property(imported gtest IMPORTED)
    if(NOT imported)
      target_compile_options(gtest PRIVATE -include cstdint)
    endif()
  endif()
endif()

ament_package()
