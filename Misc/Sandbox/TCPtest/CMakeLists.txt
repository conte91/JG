find_package(Boost COMPONENTS system thread iostreams REQUIRED)

add_executable(TCPTest main.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(TCPTest ${Boost_LIBRARIES} pthread)
