if(NOT(DEFINED PCL_FOUND))
  find_package(PCL REQUIRED)
endif(NOT(DEFINED PCL_FOUND))

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
add_executable(test_models test_model_pcl.cpp)

target_link_libraries(test_models camera icp_models ${PCL_LIBRARIES})
