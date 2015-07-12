find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(img SHARED Image.cpp ImageWMask.cpp )
target_link_libraries(img ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(img PROPERTIES COMPILE_FLAGS "-fPIC" )

add_subdirectory(Manipulation/)
