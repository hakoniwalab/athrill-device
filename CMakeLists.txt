cmake_minimum_required(VERSION 3.16)
project(athrill-device VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
include(CTest)
include(GNUInstallDirs)

add_library(athrill_device STATIC
  src/athrill_device.c
  src/models/fm25v02.c
  src/models/icm20689.c
  src/models/ist8310.c
  src/models/ms5611.c
  src/models/ublox_m8.c
  src/sha256.c
  third_party/libfdt/libfdt.c)
add_library(athrill-device::athrill-device ALIAS athrill_device)
target_include_directories(athrill_device
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE src third_party/libfdt)
target_compile_definitions(athrill_device PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
target_link_libraries(athrill_device PUBLIC ${CMAKE_DL_LIBS} m)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(TEST_DTB_DIR "${CMAKE_CURRENT_BINARY_DIR}/tests/dtb")
  add_custom_target(generate-test-dtbs
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/make_test_dtbs.py"
            "${TEST_DTB_DIR}"
    BYPRODUCTS
      "${TEST_DTB_DIR}/valid.dtb" "${TEST_DTB_DIR}/overlap.dtb"
      "${TEST_DTB_DIR}/missing-provider.dtb" "${TEST_DTB_DIR}/cycle.dtb"
      "${TEST_DTB_DIR}/plugin.dtb" "${TEST_DTB_DIR}/order.dtb"
      "${TEST_DTB_DIR}/resources.dtb" "${TEST_DTB_DIR}/wiring.dtb"
      "${TEST_DTB_DIR}/fram.dtb"
      "${TEST_DTB_DIR}/barometer.dtb"
      "${TEST_DTB_DIR}/magnetometer.dtb"
      "${TEST_DTB_DIR}/gps.dtb"
      "${TEST_DTB_DIR}/imu.dtb"
      "${TEST_DTB_DIR}/disabled-provider.dtb"
      "${TEST_DTB_DIR}/duplicate-phandle.dtb"
      "${TEST_DTB_DIR}/duplicate-spi.dtb"
      "${TEST_DTB_DIR}/duplicate-i2c.dtb" "${TEST_DTB_DIR}/bad-reg.dtb"
      "${TEST_DTB_DIR}/overflow-reg.dtb"
      "${TEST_DTB_DIR}/overflow-ranges.dtb"
      "${TEST_DTB_DIR}/truncated.dtb"
      "${TEST_DTB_DIR}/bad-string-offset.dtb"
    VERBATIM)
  add_executable(test_device_tree tests/test_device_tree.c)
  target_compile_definitions(test_device_tree PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES TEST_DTB_DIR="${TEST_DTB_DIR}")
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_device_tree PRIVATE -UNDEBUG)
  endif()
  target_link_libraries(test_device_tree PRIVATE athrill_device)
  add_dependencies(test_device_tree generate-test-dtbs)

  add_library(test_plugin_good MODULE tests/test_plugin.c)
  target_compile_definitions(test_plugin_good PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_plugin_good PRIVATE include)
  set_target_properties(test_plugin_good PROPERTIES PREFIX "")

  add_library(test_plugin_bad MODULE tests/test_plugin.c)
  target_compile_definitions(test_plugin_bad PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES TEST_PLUGIN_BAD_ABI)
  target_include_directories(test_plugin_bad PRIVATE include)
  set_target_properties(test_plugin_bad PROPERTIES PREFIX "")

  add_test(NAME device-tree-framework
    COMMAND test_device_tree $<TARGET_FILE:test_plugin_good> $<TARGET_FILE:test_plugin_bad>)
endif()

install(TARGETS athrill_device EXPORT athrill-device-targets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES include/athrill_device_if.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT athrill-device-targets NAMESPACE athrill-device::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/athrill-device")
