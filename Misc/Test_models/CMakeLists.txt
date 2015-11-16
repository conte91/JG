if(NOT(DEFINED OpenCV_FOUND))
  find_package(OpenCV REQUIRED)
endif(NOT(DEFINED OpenCV_FOUND))
if(NOT(DEFINED Eigen_FOUND))
  find_package(Eigen REQUIRED)
endif(NOT(DEFINED Eigen_FOUND))
if(NOT(DEFINED PCL_FOUND))
  find_package(PCL REQUIRED)
endif(NOT(DEFINED PCL_FOUND))

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(test_models test_model_pcl.cpp)

target_link_libraries(test_models camera icp_models ${PCL_LIBRARIES})

add_executable(test_render_model test_render_model.cpp)

target_link_libraries(test_render_model icp_models camera renderer3d ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_model_cloud test_model_cloud.cpp)

target_link_libraries(test_model_cloud icp_models camera renderer3d ${Eigen_LIBRARIES} ${OpenCV_LIBRARIES})
