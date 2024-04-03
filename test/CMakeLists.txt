include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(test test_astar.cc)
target_link_libraries(test mylib)