find_package(Boost COMPONENTS system thread REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
add_executable(test_condition_variable TestCV.cpp)

target_link_libraries(test_condition_variable ${Boost_LIBRARIES} )
