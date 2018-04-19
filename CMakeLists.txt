cmake_minimum_required(VERSION 2.8.9)
project(flood LANGUAGES C CXX)

set(RENDER FALSE CACHE BOOL "description")

if(RENDER)
	add_definitions(-DRENDER)
	add_library(STB_IMAGE "opengl/src/stb_image.cpp")
	set(LIBS ${LIBS} STB_IMAGE)
	include_directories(opengl/includes)

	add_library(GLAD "opengl/src/glad.c")
	target_link_libraries(GLAD dl)
	target_compile_options(GLAD PUBLIC -ldl)
	set(LIBS ${LIBS} GLAD)

	include_directories ("${PROJECT_SOURCE_DIR}/opengl/src")
	add_subdirectory(opengl)
endif(RENDER)

#Link Library
include_directories ("${PROJECT_SOURCE_DIR}/src")
include_directories ("${PROJECT_SOURCE_DIR}/svd")
add_subdirectory (src) 
add_subdirectory(svd)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${flood_SOURCE_DIR}/build/bin)
set(TARGET flood)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

# Add Executable
add_executable(${TARGET} src/main.cpp)
target_link_libraries(${TARGET} m)
target_link_libraries (${TARGET} src)
target_link_libraries(${TARGET} svd)
if(RENDER)
	target_link_libraries(${TARGET} render)
endif(RENDER)

add_custom_command(TARGET flood POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:flood>)