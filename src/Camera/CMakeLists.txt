find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(assimp REQUIRED)
find_package(GLUT REQUIRED)
find_package(FREEIMAGE REQUIRED)
message("OpenCV LIBRARIES: ${OpenCV_LIBRARIES}")

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${FREEIMAGE_INCLUDE_DIRS})

add_library(camera SHARED Image.cpp ImageWMask.cpp ImageViewer.cpp ImageConsumer.cpp ImageProvider.cpp DummyConsumer.cpp DummyProvider.cpp OpenniProvider.cpp OpenniWaitProvider.cpp OpenniStreamProvider.cpp FileProvider.cpp IndexTaker.cpp ClickIndexTaker.cpp Cutter.cpp) 

add_library(giorgio RecognitionData.cpp GiorgioUtils.cpp renderer3d_impl_glut.cpp linemod_icp.cpp model.cpp Renderer3d.cpp GLUTInit.cpp)

target_link_libraries(giorgio ${GLUT_LIBRARIES})

target_link_libraries(camera ${OpenCV_LIBRARIES} ${assimp_LIBRARIES} ${PCL_LIBRARIES} ${GLUT_LIBRARIES} ${FREEIMAGE_LIBRARIES} opencv_rgbd giorgio)

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( camera PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recognition Recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBRARIES})
add_subdirectory(Leap/)

#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( recognition PROPERTIES COMPILE_FLAGS "-fPIC" )
