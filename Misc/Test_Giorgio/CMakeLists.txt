if(NOT(DEFINED PCL_FOUND))
  find_package(PCL REQUIRED)
endif(NOT(DEFINED PCL_FOUND))

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_executable(test_giorgio TestGiorgio.cpp)
target_link_libraries(test_giorgio camera recognition c5g_misc img img_manipulation ${PCL_LIBRARIES})

add_executable(test_performance_giorgio test_performance_giorgio.cpp)
target_link_libraries(test_performance_giorgio camera recognition c5g_misc img img_manipulation ${PCL_LIBRARIES})
