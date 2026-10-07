cmake_minimum_required(VERSION 3.16)
project(erob_driver)

find_package(ament_cmake REQUIRED)
find_package(erob_soem_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED src/health.cpp src/config.cpp src/mock_transport.cpp src/soem_transport.cpp src/engine.cpp src/driver.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PRIVATE erob_soem_vendor::erob_soem yaml-cpp Threads::Threads)

install(TARGETS ${PROJECT_NAME} EXPORT erob_driverTargets LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(erob_driverTargets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_driver test/test_driver.cpp)
  # Humble's bundled gtest predates GCC 15's removal of transitive cstdint.
  if(TARGET gtest)
    get_target_property(gtest_imported gtest IMPORTED)
    if(NOT gtest_imported)
      target_compile_options(gtest PRIVATE -include cstdint)
    endif()
  endif()
  target_link_libraries(test_driver ${PROJECT_NAME} Threads::Threads)
  target_compile_definitions(test_driver PRIVATE EROB_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/../erob_bringup/config/robot_mock.yaml")
endif()

ament_package()
