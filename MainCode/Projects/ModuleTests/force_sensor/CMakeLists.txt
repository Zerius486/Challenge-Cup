cmake_minimum_required(VERSION 3.22)

project(Force_Sensor_Test LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(${CMAKE_CURRENT_LIST_DIR}/../common/standalone_firmware.cmake)

add_standalone_firmware(
    NAME Force_Sensor_Test
    SOURCES
      ${CMAKE_CURRENT_LIST_DIR}/Src/main.c
      ${STANDALONE_REPO_ROOT}/App/Src/force_sensor.c)
