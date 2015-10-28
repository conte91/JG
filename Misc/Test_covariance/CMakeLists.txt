MAYBE_FIND(OpenCV)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(test_covariance test_covariance.cpp)
target_link_libraries(test_covariance ${OpenCV_LIBRARIES} )
