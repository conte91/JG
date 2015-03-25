find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(test_c5g test_c5g.cpp)
target_link_libraries(test_c5g c5g c5g_misc ${Boost_LIBRARIES})
