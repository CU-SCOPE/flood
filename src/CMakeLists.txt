add_library(src kd_tree.c stl.c icp.c)
target_link_libraries(src m)