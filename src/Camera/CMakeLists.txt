find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(assimp REQUIRED)
find_package(GLUT REQUIRED)
find_package(FREEIMAGE REQUIRED)
message("OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${FREEIMAGE_INCLUDE_DIRS})

add_library(camera SHARED ImageViewer.cpp ImageConsumer.cpp ImageProvider.cpp DummyConsumer.cpp DummyProvider.cpp OpenniProvider.cpp Openni1Provider.cpp Openni2Provider.cpp OpenniStreamProvider.cpp FileProvider.cpp )

target_link_libraries(camera ${OpenCV_LIBRARIES} ${assimp_LIBRARIES} ${PCL_LIBRARIES} ${GLUT_LIBRARIES} ${FREEIMAGE_LIBRARIES} opencv_rgbd giorgio img)

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( camera PROPERTIES COMPILE_FLAGS "-fPIC" )

add_subdirectory(Leap/)
