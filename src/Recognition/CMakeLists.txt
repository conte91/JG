find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)
find_package(ASSIMP REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${freeimage_INCLUDE_DIRS})

add_library(renderer3d SHARED renderer3d_impl_glut.cpp Renderer3d.cpp Mesh.cpp GiorgioUtils.cpp GLUTInit.cpp)
target_link_libraries(renderer3d ${GLUT_LIBRARIES} freeimage ${ASSIMP_LIBRARIES})
SET_TARGET_PROPERTIES(renderer3d PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(icp_models SHARED linemod_icp.cpp Model.cpp)
SET_TARGET_PROPERTIES(icp_models PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(giorgio SHARED RecognitionData.cpp)
target_link_libraries(giorgio icp_models renderer3d c5g_misc)
SET_TARGET_PROPERTIES( giorgio PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recognition SHARED Recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBRARIES} c5g_misc robotdata)
#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( recognition PROPERTIES COMPILE_FLAGS "-fPIC" )