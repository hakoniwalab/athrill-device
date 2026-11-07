cmake_minimum_required(VERSION 3.16)
project(athrill-device VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
include(CTest)
include(GNUInstallDirs)

add_library(athrill_device STATIC
  src/channel.c
  src/protocol_dma.c
  src/protocol_i2c.c
  src/protocol_sd.c
  src/protocol_spi.c
  src/protocol_uart.c
  src/athrill_device.c
  src/models/bmi055.c
  src/models/fm25v02.c
  src/models/icm20689.c
  src/models/icm42688p.c
  src/models/ist8310.c
  src/models/ms5611.c
  src/models/pwm_actuator.c
  src/models/stm32_gpio.c
  src/models/stm32_exti.c
  src/models/stm32_tim.c
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

# Dependency-free reference plugin used by board/boot checks.  Production
# Hakoniwa and in-process variants are built from board/common and use the
# same model implementation with their respective link backends.
add_library(athrill_hakoniwa_sensor MODULE
  board/common/src/hako_sensor_device_model.c)
target_include_directories(athrill_hakoniwa_sensor PRIVATE
  include board/common/src board/common/include)
target_compile_definitions(athrill_hakoniwa_sensor PRIVATE
  ATHRILL_DEVICE_STANDALONE_TYPES ATHRILL_HAKONIWA_PASSIVE)
set_target_properties(athrill_hakoniwa_sensor PROPERTIES
  PREFIX "" OUTPUT_NAME "athrill_hakoniwa_sensor")

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  file(GLOB_RECURSE TRACE_BOARD_DTBS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/boards/*.dtb")
  set(TRACE_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/traceability")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/spi.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/dt_testgen.py")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/dt_testgen.py"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/spi.yaml"
            --out "${TRACE_GENERATED_DIR}" ${TRACE_BOARD_DTBS}
    RESULT_VARIABLE TRACE_TESTGEN_RESULT
    ERROR_VARIABLE TRACE_TESTGEN_ERROR)
  if(NOT TRACE_TESTGEN_RESULT EQUAL 0)
    message(FATAL_ERROR "SPI trace test generation failed: ${TRACE_TESTGEN_ERROR}")
  endif()
  include("${TRACE_GENERATED_DIR}/trace-tests.cmake")

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
      "${TEST_DTB_DIR}/gpio.dtb"
      "${TEST_DTB_DIR}/exti.dtb"
      "${TEST_DTB_DIR}/timer-pwm.dtb"
      "${TEST_DTB_DIR}/timer-h7-pwm.dtb"
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
  target_link_libraries(test_device_tree PRIVATE athrill_device)
  add_dependencies(test_device_tree generate-test-dtbs)

  add_executable(test_channel tests/test_channel.c)
  target_compile_definitions(test_channel PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_channel PRIVATE athrill_device)

  add_executable(test_spi_protocol tests/test_spi_protocol.c)
  target_compile_definitions(test_spi_protocol PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_spi_protocol PRIVATE athrill_device)

  add_executable(test_i2c_protocol tests/test_i2c_protocol.c)
  target_compile_definitions(test_i2c_protocol PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_i2c_protocol PRIVATE athrill_device)

  add_executable(test_uart_protocol tests/test_uart_protocol.c)
  target_compile_definitions(test_uart_protocol PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_uart_protocol PRIVATE athrill_device)

  add_executable(test_dma_protocol tests/test_dma_protocol.c)
  target_compile_definitions(test_dma_protocol PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_dma_protocol PRIVATE athrill_device)

  add_executable(test_sd_protocol tests/test_sd_protocol.c)
  target_compile_definitions(test_sd_protocol PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_sd_protocol PRIVATE athrill_device)

  add_executable(test_pixhawk4_dtb tests/test_pixhawk4_dtb.c
    third_party/libfdt/libfdt.c)
  target_include_directories(test_pixhawk4_dtb PRIVATE third_party/libfdt)
  target_compile_definitions(test_pixhawk4_dtb PRIVATE
    PIXHAWK4_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk4/pixhawk4.dtb"
    PIXHAWK4_HAKONIWA_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk4/pixhawk4-hakoniwa.dtb")

  add_executable(test_pixhawk6c_dtb tests/test_pixhawk6c_dtb.c
    third_party/libfdt/libfdt.c)
  target_include_directories(test_pixhawk6c_dtb PRIVATE third_party/libfdt)
  target_compile_definitions(test_pixhawk6c_dtb PRIVATE
    PIXHAWK6C_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk6c/pixhawk6c.dtb"
    PIXHAWK6C_HAKONIWA_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk6c/pixhawk6c-hakoniwa.dtb")

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
  add_test(NAME channel-engine COMMAND test_channel)
  add_test(NAME protocol-spi COMMAND test_spi_protocol)
  add_test(NAME protocol-i2c COMMAND test_i2c_protocol)
  add_test(NAME protocol-uart COMMAND test_uart_protocol)
  add_test(NAME protocol-dma COMMAND test_dma_protocol)
  add_test(NAME protocol-sd COMMAND test_sd_protocol)
  add_test(NAME pixhawk4-dtb COMMAND test_pixhawk4_dtb)
  add_test(NAME pixhawk6c-dtb COMMAND test_pixhawk6c_dtb)
endif()

install(TARGETS athrill_device EXPORT athrill-device-targets
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES include/athrill_device_if.h include/athrill_channel.h
  include/athrill_protocol_spi.h
  include/athrill_protocol_i2c.h
  include/athrill_protocol_sd.h
  include/athrill_protocol_uart.h
  include/athrill_protocol_dma.h
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(EXPORT athrill-device-targets NAMESPACE athrill-device::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/athrill-device")
