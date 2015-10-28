message("OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")

add_library(camera SHARED ImageViewer.cpp ImageConsumer.cpp ImageProvider.cpp DummyConsumer.cpp DummyProvider.cpp OpenniProvider.cpp Openni1Provider.cpp Openni2Provider.cpp OpenniStreamProvider.cpp FileProvider.cpp CameraModel.cpp)

target_link_libraries(camera ${OpenCV_LIBRARIES} ${assimp_LIBRARIES} ${GLUT_LIBRARIES} ${FREEIMAGE_LIBRARIES} ${PCL_LIBRARIES} opencv_rgbd giorgio img)

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( camera PROPERTIES COMPILE_FLAGS "-fPIC" )

add_subdirectory(Leap/)
