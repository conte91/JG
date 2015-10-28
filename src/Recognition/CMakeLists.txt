MAYBE_FIND(Eigen)
MAYBE_FIND(OpenCV)
MAYBE_FIND(GLUT)
MAYBE_FIND(GLEW)
MAYBE_FIND(PCL)
MAYBE_FIND(ASSIMP)

include_directories(${Eigen_INCLUDE_DIRECTORIES} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${freeimage_INCLUDE_DIRS})

add_library(linemod_with_masks SHARED DetectorWMasks.cpp)

add_library(linemod_additional_mods SHARED ColorGradientPyramidFull.cpp)
target_link_libraries(linemod_additional_mods ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(linemod_additional_mods PROPERTIES COMPILE_FLAGS "-fPIC" )


add_library(renderer3d SHARED renderer3d_impl_glut.cpp Renderer3d.cpp Mesh.cpp GLUTInit.cpp)
target_link_libraries(renderer3d ${GLUT_LIBRARIES} freeimage ${ASSIMP_LIBRARIES} ${GLEW_LIBRARIES} linemod_with_masks)
SET_TARGET_PROPERTIES(renderer3d PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(icp_models SHARED linemod_icp.cpp Model.cpp)
target_link_libraries(icp_models linemod_additional_mods)
SET_TARGET_PROPERTIES(icp_models PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(giorgio SHARED RecognitionData.cpp)
target_link_libraries(giorgio icp_models renderer3d c5g_misc ${PCL_LIBRARIES} linemod_additional_mods)
SET_TARGET_PROPERTIES( giorgio PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recognition SHARED Recognition.cpp)
target_link_libraries(recognition ${OpenCV_LIBRARIES} c5g_misc robotdata linemod_additional_mods)
#Necessary as this library will be linked to a shared object later
SET_TARGET_PROPERTIES( recognition PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(recogUtils SHARED Utils.cpp)
target_link_libraries(recogUtils ${Eigen_LIBRARIES})
SET_TARGET_PROPERTIES( recogUtils PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(points_iterators SHARED SphereSplitter.cpp)
target_link_libraries(points_iterators ${Eigen_LIBRARIES})
SET_TARGET_PROPERTIES( points_iterators PROPERTIES COMPILE_FLAGS "-fPIC" )
