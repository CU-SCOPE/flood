add_library(svd svdcmp.c)
target_link_libraries(svd m)
target_compile_options(svd PUBLIC -O3 -fPIC)