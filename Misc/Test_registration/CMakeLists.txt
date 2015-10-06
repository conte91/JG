MAYBE_FIND(PCL)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
add_executable( test_registration test_registration.cpp )
target_link_libraries( test_registration ${PCL_LIBRARIES} )
