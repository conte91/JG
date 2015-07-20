find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_LIBRARIES})

add_executable(test_depth test_depth.cpp)
target_link_libraries(test_depth ${OpenCV_LIBRARIES} camera)
