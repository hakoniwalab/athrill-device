cmake_minimum_required(VERSION 3.20)

# Hakoniwa sensor device for Athrill (an EXDEV shared library), see
# README.md.
project(hako-athrill-sensor-device LANGUAGES C)

# The Athrill headers are C11 with GNU extensions. GCC 15 defaults to C23, where
# bool is a keyword and Athrill's std_types.h typedef no longer compiles.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

option(HAKO_SENSOR_DEVICE_BUILD_INPROC
  "Build the sensor device with a Hakoniwa Drone aircraft in process" OFF)

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
  src/hako_sensor_device_link.c
  src/hako_sensor_device_link_mavlink.c)
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

if(HAKO_SENSOR_DEVICE_BUILD_INPROC)
  enable_language(CXX)
  set(HAKO_DRONE_ROOT "" CACHE PATH "Hakoniwa Drone v5 release root")
  set(HAKO_DRONE_SOURCE_DIR "" CACHE PATH
      "Matching Drone source tree (header fallback for files omitted from the v5 release)")
  if(NOT EXISTS "${HAKO_DRONE_ROOT}/include/service/aircraft/iaircraft_service_container.hpp")
    message(FATAL_ERROR "Set HAKO_DRONE_ROOT to the unpacked Hakoniwa Drone v5 release")
  endif()
  if(NOT EXISTS "${HAKO_DRONE_SOURCE_DIR}/src/aircraft/impl/fault_injection/fault_rotor.hpp")
    message(FATAL_ERROR "Set HAKO_DRONE_SOURCE_DIR to the matching Drone v5 source tree")
  endif()
  if(APPLE)
    set(HAKO_DRONE_PLATFORM_DIR "${HAKO_DRONE_ROOT}/mac")
    set(HAKO_DRONE_SERVICE_LIBRARY "${HAKO_DRONE_PLATFORM_DIR}/libhako_service_c.dylib")
    set(HAKO_DRONE_MUJOCO_LIBRARY "${HAKO_DRONE_PLATFORM_DIR}/libmujoco.3.16.0.dylib")
  else()
    message(FATAL_ERROR "The in-process prototype currently supports the macOS v5 release")
  endif()
  if(NOT EXISTS "${HAKO_DRONE_SERVICE_LIBRARY}" OR NOT EXISTS "${HAKO_DRONE_MUJOCO_LIBRARY}")
    message(FATAL_ERROR "Hakoniwa Drone release libraries were not found under ${HAKO_DRONE_PLATFORM_DIR}")
  endif()

  function(configure_hako_inproc_target target)
    target_link_libraries(${target} PRIVATE "${HAKO_DRONE_SERVICE_LIBRARY}")
    target_include_directories(${target} PRIVATE
      "${HAKO_DRONE_ROOT}/include"
      "${HAKO_DRONE_SOURCE_DIR}/include"
      "${HAKO_DRONE_SOURCE_DIR}/include/aircraft/interfaces"
      "${HAKO_DRONE_SOURCE_DIR}/src"
      "${HAKO_DRONE_SOURCE_DIR}/thirdparty/nolman/single_include"
      "${HAKO_DRONE_SOURCE_DIR}/thirdparty/glm"
      "${HAKO_PDU_REGISTRY_DIR}/pdu/types")
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_compile_definitions(${target} PRIVATE HAKO_ENABLE_SENSOR_MODEL_V2)
    if(APPLE)
      # Both release dylibs use @loader_path, so keep them beside each consumer.
      add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${HAKO_DRONE_SERVICE_LIBRARY}" "$<TARGET_FILE_DIR:${target}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${HAKO_DRONE_MUJOCO_LIBRARY}" "$<TARGET_FILE_DIR:${target}>")
    endif()
  endfunction()

  add_library(hako_sensor_device_inproc SHARED
    src/hako_sensor_device_exdev.c
    src/hako_sensor_device_link_inproc.cpp
    src/hako_sensor_device_link_mavlink.c)
  target_link_libraries(hako_sensor_device_inproc PRIVATE hako_sensor_device_core)
  target_include_directories(hako_sensor_device_inproc PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:${ATHRILL_SRC_DIR}/inc>"
    "$<$<COMPILE_LANGUAGE:C>:${ATHRILL_SRC_DIR}/lib>"
    "$<$<COMPILE_LANGUAGE:C>:${ATHRILL_SRC_DIR}/cpu>"
    "$<$<COMPILE_LANGUAGE:C>:${ATHRILL_SRC_DIR}/device/mpu>"
    "$<$<COMPILE_LANGUAGE:C>:${ATHRILL_SRC_DIR}/device/peripheral/serial/fifo>")
  configure_hako_inproc_target(hako_sensor_device_inproc)
  target_compile_definitions(hako_sensor_device_inproc PRIVATE
    HAKO_SENSOR_DEVICE_INPROC
    ATHRILL_EXT_DEVICE OS_LINUX ATHRILL_ADDRESS_WIDTH=${ATHRILL_ADDRESS_WIDTH})
  set_target_properties(hako_sensor_device_inproc PROPERTIES
    C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden)
  if(APPLE)
    target_link_options(hako_sensor_device_inproc PRIVATE -undefined dynamic_lookup)
  endif()

  add_executable(hako_sensor_inproc_test
    test/hako_sensor_inproc_test.cpp
    src/hako_sensor_device_link_inproc.cpp
    src/hako_sensor_device_link_mavlink.c)
  target_include_directories(hako_sensor_inproc_test PRIVATE src include)
  configure_hako_inproc_target(hako_sensor_inproc_test)

  add_executable(hako_sensor_hakoniwa_test
    test/hako_sensor_inproc_test.cpp
    src/hako_sensor_device_link.c
    src/hako_sensor_device_link_mavlink.c)
  target_link_libraries(hako_sensor_hakoniwa_test PRIVATE hakoniwa-core::shakoc)
  target_include_directories(hako_sensor_hakoniwa_test PRIVATE
    src include "${HAKO_PDU_REGISTRY_DIR}/pdu/types")
  target_compile_features(hako_sensor_hakoniwa_test PRIVATE cxx_std_17)
endif()
