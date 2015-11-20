find_package(OpenCV REQUIRED)

add_executable(OpenNI_save_images main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(OpenNI_save_images ${OpenCV_LIBRARIES} camera)
