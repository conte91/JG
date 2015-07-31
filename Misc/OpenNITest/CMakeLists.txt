find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(openni_test OpenniProvider.cpp)

target_link_libraries(openni_test ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
