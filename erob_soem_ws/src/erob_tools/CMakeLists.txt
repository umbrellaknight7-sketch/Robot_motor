cmake_minimum_required(VERSION 3.16)
project(erob_tools)

find_package(ament_cmake REQUIRED)
find_package(erob_driver REQUIRED)

add_executable(erob_inspect src/erob_inspect.cpp)
target_compile_features(erob_inspect PRIVATE cxx_std_17)
target_link_libraries(erob_inspect erob_driver::erob_driver)

install(TARGETS erob_inspect DESTINATION lib/${PROJECT_NAME})

ament_package()
