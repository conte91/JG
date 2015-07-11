find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
add_library(giorgio RecognitionData.cpp GiorgioUtils.cpp renderer3d_impl_glut.cpp linemod_icp.cpp model.cpp Renderer3d.cpp GLUTInit.cpp)
target_link_libraries(giorgio ${GLUT_LIBRARIES})
SET_TARGET_PROPERTIES( giorgio PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recognition Recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBRARIES})
#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( recognition PROPERTIES COMPILE_FLAGS "-fPIC" )
