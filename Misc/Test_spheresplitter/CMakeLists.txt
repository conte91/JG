MAYBE_FIND(PCL)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
add_executable( test_spheresplitter test_spheresplitter.cpp )
target_link_libraries( test_spheresplitter points_iterators ${PCL_LIBRARIES} )
