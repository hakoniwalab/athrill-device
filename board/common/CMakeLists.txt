cmake_minimum_required(VERSION 3.20)

# Hakoniwa sensor device for Athrill (an EXDEV shared library), see
# README.md.
project(hako-athrill-sensor-device LANGUAGES C)

set(ATHRILL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../athrill"
    CACHE PATH "Athrill core source tree (toppers/athrill v3.0.0 or later)")
get_filename_component(ATHRILL_SOURCE_DIR "${ATHRILL_SOURCE_DIR}" ABSOLUTE)
set(ATHRILL_SRC_DIR "${ATHRILL_SOURCE_DIR}/src")
if(NOT EXISTS "${ATHRILL_SRC_DIR}/inc/athrill_exdev.h")
  message(FATAL_ERROR "Athrill EXDEV headers not found under ${ATHRILL_SOURCE_DIR}")
endif()

# The register behaviour, without Athrill (also used by tests).
add_library(hako_sensor_device_core STATIC src/hako_sensor_device.c)
target_include_directories(hako_sensor_device_core PUBLIC src include)
set_target_properties(hako_sensor_device_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Hakoniwa core (polling asset API) and the PDU C types.
find_package(hakoniwa-core CONFIG REQUIRED)
set(HAKO_PDU_REGISTRY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../hakoniwa-pdu-registry"
    CACHE PATH "hakoniwa-pdu-registry source tree")

# The EXDEV loaded by athrill3 (memory.txt: DEV, 0x20000000, <this library>).
add_library(hako_sensor_device SHARED
  src/hako_sensor_device_exdev.c
  src/hako_sensor_device_link.c)
target_link_libraries(hako_sensor_device PRIVATE hako_sensor_device_core hakoniwa-core::shakoc)
target_include_directories(hako_sensor_device PRIVATE "${HAKO_PDU_REGISTRY_DIR}/pdu/types")
target_include_directories(hako_sensor_device PRIVATE
  "${ATHRILL_SRC_DIR}/inc"
  "${ATHRILL_SRC_DIR}/lib"
  "${ATHRILL_SRC_DIR}/cpu"
  "${ATHRILL_SRC_DIR}/device/mpu"
  "${ATHRILL_SRC_DIR}/device/peripheral/serial/fifo")
# Must match the athrill3 that loads the device: the RISC-V target uses a
# 64-bit bus, and AthrillAddrType is part of the EXDEV structures.
set(ATHRILL_ADDRESS_WIDTH 64 CACHE STRING "Address width of the athrill3 that loads the device (32 or 64)")
target_compile_definitions(hako_sensor_device PRIVATE
  ATHRILL_EXT_DEVICE OS_LINUX ATHRILL_ADDRESS_WIDTH=${ATHRILL_ADDRESS_WIDTH})
set_target_properties(hako_sensor_device PROPERTIES C_VISIBILITY_PRESET hidden)
if(APPLE)
  # Athrill provides the EXDEV interface symbols at load time.
  target_link_options(hako_sensor_device PRIVATE -undefined dynamic_lookup)
endif()
