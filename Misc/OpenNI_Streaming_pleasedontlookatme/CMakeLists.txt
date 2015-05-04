find_package(OpenCV REQUIRED)

add_executable(OpenNI_Streamer main.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(OpenNI_Streamer ${OpenCV_LIBRARIES})
