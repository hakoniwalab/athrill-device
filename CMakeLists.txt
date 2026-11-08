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
  src/bindings.c
  src/bindings_v1.c
  src/models/stm32_gpio.c
  src/models/stm32_exti.c
  src/models/stm32_dmamux.c
  src/models/stm32_dma.c
  src/models/stm32_sdmmc.c
  src/models/stm32h7_sdmmc.c
  src/models/sdcard.c
  src/models/sdcard_card.c
  src/models/register_block.c
  src/models/riscv_mtimer.c
  src/models/shakti_uart.c
  src/models/shakti_uart_tcp.c
  src/models/bench_sensor.c
  src/models/stm32_i2c.c
  src/models/stm32_spi.c
  src/models/stm32_spi_bindings.c
  src/models/stm32_spi_bus.c
  src/models/stm32h7_spi.c
  src/models/stm32_uart.c
  src/models/stm32_uart_tcp.c
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
# Values the SVD does not hold (#32 rule 3), one header per model in
# targets/stm32/config; replace them with your own headers through
# ATHRILL_STM32_CONFIG_DIR (searched first).
set(ATHRILL_STM32_CONFIG_DIR "" CACHE PATH "Directory with user model configuration headers (stm32_*_config.h)")
if(ATHRILL_STM32_CONFIG_DIR)
  get_filename_component(_athrill_stm32_config "${ATHRILL_STM32_CONFIG_DIR}" ABSOLUTE)
  target_include_directories(athrill_device PRIVATE "${_athrill_stm32_config}")
endif()
target_include_directories(athrill_device PRIVATE targets/stm32/config)
# Bindings (spec/bindings/*_gen.yaml) are read at start-up with libyaml.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED IMPORTED_TARGET yaml-0.1)
set(ATHRILL_DEVICE_BINDINGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/spec/bindings" CACHE PATH
  "Default directory of the generated bindings (override at run time with ATHRILL_DEVICE_BINDINGS_DIR)")
target_compile_definitions(athrill_device PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES
  ATHRILL_DEVICE_BINDINGS_DEFAULT_DIR="${ATHRILL_DEVICE_BINDINGS_DIR}")
find_package(Threads REQUIRED)
target_link_libraries(athrill_device PUBLIC ${CMAKE_DL_LIBS} m PkgConfig::YAML Threads::Threads)

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
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/i2c.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/uart.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/dt.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/dma.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/sd.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/pwm.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/gpio.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/spec/chips.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/dt_testgen.py")
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/dt_testgen.py"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/spi.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/i2c.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/uart.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/dt.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/dma.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/sd.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/pwm.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/gpio.yaml"
            --spec "${CMAKE_CURRENT_SOURCE_DIR}/spec/chips.yaml"
            --out "${TRACE_GENERATED_DIR}" ${TRACE_BOARD_DTBS}
    RESULT_VARIABLE TRACE_TESTGEN_RESULT
    ERROR_VARIABLE TRACE_TESTGEN_ERROR)
  if(NOT TRACE_TESTGEN_RESULT EQUAL 0)
    message(FATAL_ERROR "bus trace test generation failed: ${TRACE_TESTGEN_ERROR}")
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
      "${TEST_DTB_DIR}/dma-request.dtb"
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

  add_executable(test_bindings tests/test_bindings.c)
  target_compile_definitions(test_bindings PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_link_libraries(test_bindings PRIVATE athrill_device)
  add_test(NAME bindings COMMAND test_bindings)
  add_executable(test_bindings_conformance tests/test_bindings_conformance.c)
  target_compile_definitions(test_bindings_conformance PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES
    ATHRILL_TEST_BINDINGS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/spec/bindings")
  target_link_libraries(test_bindings_conformance PRIVATE athrill_device PkgConfig::YAML)
  add_test(NAME bindings-conformance COMMAND test_bindings_conformance)

  add_executable(test_stm32_dmamux tests/test_stm32_dmamux.c)
  target_compile_definitions(test_stm32_dmamux PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_dmamux PRIVATE src/models)
  target_link_libraries(test_stm32_dmamux PRIVATE athrill_device)
  add_test(NAME stm32-dmamux COMMAND test_stm32_dmamux)
  add_executable(test_stm32_dma tests/test_stm32_dma.c)
  target_compile_definitions(test_stm32_dma PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_dma PRIVATE src/models)
  target_link_libraries(test_stm32_dma PRIVATE athrill_device)
  add_test(NAME stm32-dma COMMAND test_stm32_dma)
  add_executable(test_stm32_sdmmc tests/test_stm32_sdmmc.c)
  target_compile_definitions(test_stm32_sdmmc PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_sdmmc PRIVATE src/models)
  target_link_libraries(test_stm32_sdmmc PRIVATE athrill_device)
  add_test(NAME stm32-sdmmc COMMAND test_stm32_sdmmc)
  add_executable(test_register_block tests/test_register_block.c)
  target_compile_definitions(test_register_block PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_register_block PRIVATE src/models)
  target_link_libraries(test_register_block PRIVATE athrill_device)
  add_test(NAME register-block COMMAND test_register_block)
  add_executable(test_riscv_mtimer tests/test_riscv_mtimer.c)
  target_compile_definitions(test_riscv_mtimer PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_riscv_mtimer PRIVATE src/models)
  target_link_libraries(test_riscv_mtimer PRIVATE athrill_device)
  add_test(NAME riscv-mtimer COMMAND test_riscv_mtimer)
  add_executable(test_shakti_uart tests/test_shakti_uart.c)
  target_compile_definitions(test_shakti_uart PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_shakti_uart PRIVATE src/models)
  target_link_libraries(test_shakti_uart PRIVATE athrill_device)
  add_test(NAME shakti-uart COMMAND test_shakti_uart)
  add_executable(test_board_register_blocks tests/test_board_register_blocks.c)
  target_compile_definitions(test_board_register_blocks PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES
    PIXHAWK4_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk4/pixhawk4.dtb"
    PIXHAWK6C_DTB="${CMAKE_CURRENT_SOURCE_DIR}/boards/pixhawk6c/pixhawk6c.dtb")
  target_include_directories(test_board_register_blocks PRIVATE src/models third_party/libfdt)
  target_link_libraries(test_board_register_blocks PRIVATE athrill_device)
  add_test(NAME board-register-blocks COMMAND test_board_register_blocks)
  add_executable(test_stm32_dma_spec tests/test_stm32_dma_spec.c)
  target_compile_definitions(test_stm32_dma_spec PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_dma_spec PRIVATE src/models)
  target_link_libraries(test_stm32_dma_spec PRIVATE athrill_device)
  add_test(NAME stm32-dma-spec COMMAND test_stm32_dma_spec)

  add_executable(test_stm32_i2c tests/test_stm32_i2c.c)
  target_compile_definitions(test_stm32_i2c PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_i2c PRIVATE src/models)
  target_link_libraries(test_stm32_i2c PRIVATE athrill_device)
  add_test(NAME stm32-i2c COMMAND test_stm32_i2c)

  add_executable(test_stm32_spi tests/test_stm32_spi.c)
  target_compile_definitions(test_stm32_spi PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_spi PRIVATE src/models)
  target_link_libraries(test_stm32_spi PRIVATE athrill_device)
  add_test(NAME stm32-spi COMMAND test_stm32_spi)
  # H7 SPI behaviour the model does not implement yet (RM0433 TSIZE, CFG
  # write protection, FIFO threshold, DMA requests): documented, WILL_FAIL.
  add_test(NAME stm32h7-spi-spec-gaps COMMAND test_stm32_spi spec-gaps)
  set_tests_properties(stm32h7-spi-spec-gaps PROPERTIES WILL_FAIL TRUE)

  add_executable(test_stm32_uart tests/test_stm32_uart.c)
  target_compile_definitions(test_stm32_uart PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_stm32_uart PRIVATE src/models)
  target_link_libraries(test_stm32_uart PRIVATE athrill_device)
  add_test(NAME stm32-uart COMMAND test_stm32_uart)
  add_test(NAME stm32-uart-spec-gaps COMMAND test_stm32_uart spec-gaps)
  set_tests_properties(stm32-uart-spec-gaps PROPERTIES WILL_FAIL TRUE)

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

  add_library(test_plugin_old_minor MODULE tests/test_plugin.c)
  target_compile_definitions(test_plugin_old_minor PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES TEST_PLUGIN_OLD_MINOR)
  target_include_directories(test_plugin_old_minor PRIVATE include)
  set_target_properties(test_plugin_old_minor PROPERTIES PREFIX "")

  add_library(test_plugin_dma MODULE tests/test_plugin_dma.c)
  target_compile_definitions(test_plugin_dma PRIVATE ATHRILL_DEVICE_STANDALONE_TYPES)
  target_include_directories(test_plugin_dma PRIVATE include)
  set_target_properties(test_plugin_dma PROPERTIES PREFIX "")
  add_executable(test_dma_request tests/test_dma_request.c)
  target_compile_definitions(test_dma_request PRIVATE
    ATHRILL_DEVICE_STANDALONE_TYPES TEST_DTB_DIR="${TEST_DTB_DIR}")
  target_link_libraries(test_dma_request PRIVATE athrill_device)
  add_dependencies(test_dma_request generate-test-dtbs)
  add_test(NAME dma-request-lines
    COMMAND test_dma_request $<TARGET_FILE:test_plugin_dma>)

  add_test(NAME device-tree-framework
    COMMAND test_device_tree $<TARGET_FILE:test_plugin_good> $<TARGET_FILE:test_plugin_bad>
      $<TARGET_FILE:test_plugin_old_minor>)
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
