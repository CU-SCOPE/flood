cmake_minimum_required(VERSION 2.8.9)
project(flood LANGUAGES C CXX)

#Link Library
include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/svd")
add_subdirectory (src) 
add_subdirectory(svd)
if(CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${flood_SOURCE_DIR}/build/exe/rpi_target)
	set(TARGET rpiExe)
else(CMAKE_TOOLCHAIN_FILE)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${flood_SOURCE_DIR}/build/exe/pc_target)
	set(TARGET pcExe)
endif(CMAKE_TOOLCHAIN_FILE)

# Add Executable
add_executable(${TARGET} src/main.c)
target_link_libraries(${TARGET} m)
target_link_libraries (${TARGET} src)
target_link_libraries(${TARGET} svd)