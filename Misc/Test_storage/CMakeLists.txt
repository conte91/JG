FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
add_executable(test_storage_camera_model test_storage_camera_model.cpp)
target_link_libraries(test_storage_camera_model ${OpenCV_LIBRARIES} camera)
