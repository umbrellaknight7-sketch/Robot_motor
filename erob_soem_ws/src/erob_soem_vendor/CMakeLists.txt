cmake_minimum_required(VERSION 3.16)
project(erob_soem_vendor C)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
file(GLOB SOEM_SOURCES CONFIGURE_DEPENDS vendor/SOEM/soem/*.c)

add_library(erob_soem SHARED ${SOEM_SOURCES}
  vendor/SOEM/osal/linux/osal.c vendor/SOEM/oshw/linux/oshw.c vendor/SOEM/oshw/linux/nicdrv.c)
target_include_directories(erob_soem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/SOEM/soem>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/SOEM/osal>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/SOEM/osal/linux>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/SOEM/oshw/linux>
  $<INSTALL_INTERFACE:include/erob_soem>)
target_link_libraries(erob_soem PRIVATE Threads::Threads rt)

install(TARGETS erob_soem EXPORT erob_soem_vendorTargets LIBRARY DESTINATION lib)
install(DIRECTORY vendor/SOEM/soem/ vendor/SOEM/osal/ vendor/SOEM/osal/linux/ vendor/SOEM/oshw/linux/
  DESTINATION include/erob_soem FILES_MATCHING PATTERN "*.h" PATTERN "win32" EXCLUDE
  PATTERN "intime" EXCLUDE PATTERN "rtems" EXCLUDE PATTERN "rtk" EXCLUDE PATTERN "vxworks" EXCLUDE)
install(FILES vendor/SOEM/LICENSE UPSTREAM.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(erob_soem_vendorTargets HAS_LIBRARY_TARGET)

ament_package()
