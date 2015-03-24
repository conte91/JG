find_package(Boost COMPONENTS system thread REQUIRED)

add_executable(provamain main.cpp APC.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(provamain apc camera c5g c5g_misc
  #    log
  ${Boost_LIBRARIES})

add_subdirectory(pyModule)
