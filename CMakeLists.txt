cmake_minimum_required(VERSION 2.8.9)
project(flood LANGUAGES C CXX)

add_library(STB_IMAGE "opengl/src/stb_image.cpp")
set(LIBS ${LIBS} STB_IMAGE)
include_directories(opengl/includes)

add_library(GLAD "opengl/src/glad.c")
target_link_libraries(GLAD dl)
target_compile_options(GLAD PUBLIC -ldl)
set(LIBS ${LIBS} GLAD)

#Link Library
include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/svd")
include_directories ("${PROJECT_SOURCE_DIR}/opengl/src")
add_subdirectory (src) 
add_subdirectory(svd)
add_subdirectory(opengl)
if(CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${flood_SOURCE_DIR}/build/exe/rpi_target)
	set(TARGET rpiExe)
else(CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${flood_SOURCE_DIR}/build/exe/pc_target)
	set(TARGET pcExe)
endif(CMAKE_TOOLCHAIN_FILE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Add Executable
add_executable(${TARGET} src/main.cpp)
target_link_libraries(${TARGET} m)
target_link_libraries (${TARGET} src)
target_link_libraries(${TARGET} svd)
target_link_libraries(${TARGET} render)