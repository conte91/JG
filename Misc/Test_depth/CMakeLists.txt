find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(test_depth test_depth.cpp)
add_executable(test_xyz test_xyz.cpp)
add_executable(capture_frame capture_frame.cpp)
target_link_libraries(capture_frame ${OpenCV_LIBRARIES} camera)
target_link_libraries(test_depth ${OpenCV_LIBRARIES} camera)
target_link_libraries(test_xyz ${OpenCV_LIBRARIES} camera)
