add_library(src kd_tree.c stl.c icp.c flood.cpp image.cpp)
target_link_libraries (src ${CMAKE_THREAD_LIBS_INIT})
find_package(Threads)
target_link_libraries(src m)
if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(src PUBLIC DEBUG=1)
    target_compile_options(src PUBLIC -ggdb -std=c++11)
else(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(src PUBLIC DEBUG=0)
    target_compile_options(src PUBLIC -O3 -std=c++11 -pthread -lglog)
endif(CMAKE_BUILD_TYPE MATCHES Debug)

set(CMAKE_MODULE_PATH
    /home/zach/Documents/senior_projects/libo3d3xx/cmake/modules
    ${CMAKE_MODULE_PATH}
    )

################################################
## Bring in dependent projects
################################################
include(o3d3xx_version)
find_package(o3d3xx_camera ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(o3d3xx_framegrabber ${O3D3XX_VERSION_STRING} EXACT REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_library(LIB_boost_system NAMES boost_system)

include_directories(
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_CAMERA_LIBRARY_DIR}
  ${O3D3XX_FRAMEGRABBER_LIBRARY_DIR}
  )

target_link_libraries(src
                      ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${LIB_boost_system}
                      )