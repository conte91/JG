find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(testABRT main.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(testABRT c5g c5g_misc robotdata ${Boost_LIBRARIES})
