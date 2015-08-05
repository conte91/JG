find_package(OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS system filesystem thread REQUIRED )
find_package(PCL REQUIRED)
find_package(assimp REQUIRED)

include_directories( ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE(assimp REQUIRED)
INCLUDE_DIRECTORIES(${ASSIMP_INCLUDE_DIRS})

add_executable( LineModTraining linemod_train.cpp )

target_link_libraries( LineModTraining renderer3d ${Boost_LIBRARIES} icp_models ${OpenCV_LIBRARIES} camera)

add_executable( test_renderer Test_renderer.cpp )
target_link_libraries( test_renderer renderer3d camera)
