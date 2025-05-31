cmake_minimum_required(VERSION 3.10.0)
project(WGS84 VERSION 0.1.0 LANGUAGES C)

include(CTest)
enable_testing()
include_directories(include Unity/src)


link_libraries(m)

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(Unity)


