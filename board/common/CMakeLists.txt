cmake_minimum_required(VERSION 3.20)

# Hakoniwa sensor device for Athrill (a Devicetree model plugin).
project(hako-athrill-sensor-device LANGUAGES C)

# The Athrill headers are C11 with GNU extensions. GCC 15 defaults to C23, where
# bool is a keyword and Athrill's std_types.h typedef no longer compiles.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

option(HAKO_SENSOR_DEVICE_BUILD_INPROC
  "Build the sensor device with a Hakoniwa Drone aircraft in process" OFF)

# The register behaviour, without Athrill (also used by tests).
add_library(hako_sensor_device_core STATIC src/hako_sensor_device.c)
target_include_directories(hako_sensor_device_core PUBLIC src include)
set_target_properties(hako_sensor_device_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Hakoniwa core (polling asset API) and the PDU C types.
find_package(hakoniwa-core CONFIG REQUIRED)
set(HAKO_PDU_REGISTRY_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../hakoniwa-pdu-registry"
    CACHE PATH "hakoniwa-pdu-registry source tree")

# Explicit plugin for compatible hakoniwa,sensor-device.
add_library(hako_sensor_device SHARED
  src/hako_sensor_device_model.c
  src/hako_sensor_device_link.c
  src/hako_sensor_device_link_mavlink.c)
target_include_directories(hako_sensor_device PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/../../include"
  "${CMAKE_CURRENT_LIST_DIR}/include"
  "${HAKO_PDU_REGISTRY_DIR}/pdu/types")
target_link_libraries(hako_sensor_device PRIVATE hakoniwa-core::shakoc)
target_compile_definitions(hako_sensor_device PRIVATE
  ATHRILL_DEVICE_STANDALONE_TYPES)
set_target_properties(hako_sensor_device PROPERTIES C_VISIBILITY_PRESET hidden)

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
    src/hako_sensor_device_model.c
    src/hako_sensor_device_link_inproc.cpp
    src/hako_sensor_device_link_mavlink.c)
  target_include_directories(hako_sensor_device_inproc PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../include"
    "${CMAKE_CURRENT_LIST_DIR}/include")
  configure_hako_inproc_target(hako_sensor_device_inproc)
  target_compile_definitions(hako_sensor_device_inproc PRIVATE
    HAKO_SENSOR_DEVICE_INPROC ATHRILL_DEVICE_STANDALONE_TYPES)
  set_target_properties(hako_sensor_device_inproc PROPERTIES
    C_VISIBILITY_PRESET hidden CXX_VISIBILITY_PRESET hidden)

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
