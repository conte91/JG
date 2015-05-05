find_package(Boost COMPONENTS system program_options thread REQUIRED)

add_executable(provamain main.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(provamain apc apc_main camera c5g c5g_misc robotdata ${Boost_LIBRARIES})

add_subdirectory(pyModule)
