find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(test_scanbins test_scanbins.cpp)
target_link_libraries(test_scanbins c5g c5g_misc ${Boost_LIBRARIES} camera apc )
