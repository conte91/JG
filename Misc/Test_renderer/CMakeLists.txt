MAYBE_FIND(OpenCV)

add_executable( test_renderer Test_renderer.cpp )
target_link_libraries( test_renderer renderer3d camera ${OpenCV_LIBRARIES})
add_executable( test_transformations test_renderer_transformations.cpp )
target_link_libraries( test_transformations renderer3d camera ${OpenCV_LIBRARIES} recogUtils icp_models)
