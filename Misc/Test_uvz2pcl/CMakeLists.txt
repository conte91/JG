MAYBE_FIND(OpenCV)
MAYBE_FIND(PCL)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable( test_uvz2pcl test_uvz2pcl.cpp )
target_link_libraries( test_uvz2pcl camera ${OpenCV_LIBRARIES} )
