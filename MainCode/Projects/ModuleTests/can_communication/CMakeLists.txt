cmake_minimum_required(VERSION 3.22)

project(CAN_Communication_Test LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(${CMAKE_CURRENT_LIST_DIR}/../common/standalone_firmware.cmake)

add_standalone_firmware(
    NAME CAN_Communication_Test
    SOURCES
      ${CMAKE_CURRENT_LIST_DIR}/Src/main.c)
