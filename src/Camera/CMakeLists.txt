find_package(OpenCV REQUIRED)
message("OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(camera Image.cpp CameraView.cpp ImageConsumer.cpp ImageProvider.cpp DummyConsumer.cpp DummyProvider.cpp)

target_link_libraries(camera ${OpenCV_LIBRARIES})

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( camera PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recognition Recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBRARIES})
add_subdirectory(Leap/)

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( recognition PROPERTIES COMPILE_FLAGS "-fPIC" )
