link_libraries(unity WGS84)
add_executable(WGS84_test test.c)

add_test(test1 WGS84_test)