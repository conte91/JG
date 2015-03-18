find_package(OpenCV REQUIRED)
message("OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(camera CameraView.cpp ImageConsumer.cpp ImageProvider.cpp)

target_link_libraries(camera ${OpenCV_LIBRARIES})

add_subdirectory(Leap/)
