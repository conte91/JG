MAYBE_FIND(OpenCV)
MAYBE_FIND(OpenGL)
MAYBE_FIND(GLUT)
MAYBE_FIND(PCL)
MAYBE_FIND(assimp)
FIND_PACKAGE( Boost 1.40 COMPONENTS system filesystem thread REQUIRED )

include_directories( ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(assimp REQUIRED)
INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIRS})

add_executable( LineModTraining linemod_train.cpp )

target_link_libraries( LineModTraining renderer3d ${Boost_LIBRARIES} icp_models ${OpenCV_LIBRARIES} camera points_iterators recogUtils)
