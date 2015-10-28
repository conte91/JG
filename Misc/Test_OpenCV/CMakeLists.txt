find_package(OpenCV REQUIRED )

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(test_erode test_erode.cpp )
target_link_libraries(test_erode ${OpenCV_LIBRARIES} )

add_executable(test_downsample test_downsample.cpp )
target_link_libraries(test_downsample ${OpenCV_LIBRARIES} )

add_executable(test_flatten_matrix test_flatten_matrix.cpp )
target_link_libraries(test_flatten_matrix ${OpenCV_LIBRARIES} )
