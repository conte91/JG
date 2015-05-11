include_directories(${Boost_INCLUDE_DIRS})
add_executable(test_giorgio TestGiorgio.cpp)

target_link_libraries(test_giorgio camera recognition c5g_misc)
