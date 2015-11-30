MAYBE_FIND(PCL)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
add_executable( test_grasp_generation test_grasp_generation.cpp )
target_link_libraries( test_grasp_generation points_iterators ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} shapes grasping )

add_executable( test_single_grasp test_single_grasp.cpp )
target_link_libraries( test_single_grasp points_iterators ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} shapes grasping gripper )

add_executable( test_single_grasp_moving test_single_grasp_moving.cpp )
target_link_libraries( test_single_grasp_moving points_iterators ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} shapes grasping gripper c5g_misc)

add_executable( test_multiple_grasps test_multiple_grasps.cpp )
target_link_libraries( test_multiple_grasps points_iterators ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} shapes grasping gripper c5g_misc)
