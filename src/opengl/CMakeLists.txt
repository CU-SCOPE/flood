cmake_minimum_required(VERSION 2.8.9)
project(opengl LANGUAGES C CXX)

find_package(glfw3)
add_executable(render render.cpp glad.c stl.c)
target_compile_options(render PUBLIC -O3 -std=c++11 -ldl)
target_link_libraries(render glfw)
target_link_libraries(render dl)