cmake_minimum_required(VERSION 2.8.9)
project(Orientation)

include_directories(inc)
file(GLOB SOURCES "../../src/*.c")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../exe/pc_target")
SET(CMAKE_CXX_FLAGS "-g")

add_executable(pcExe ${SOURCES})
