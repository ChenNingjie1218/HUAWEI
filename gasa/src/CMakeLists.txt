
add_executable(mainparam main.cpp)
target_link_libraries(mainparam mylib)

add_executable(gasa gasamain.cpp)
aux_source_directory(. GASA)
list(REMOVE_ITEM GASA "./main.cpp")
add_library(gasalib ${GASA})
target_link_libraries(gasa gasalib)
