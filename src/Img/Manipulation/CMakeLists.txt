find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(img_manipulation SHARED Cutter.cpp IndexTaker.cpp ClickIndexTaker.cpp GridIndexTaker.cpp )
target_link_libraries(img_manipulation ${OpenCV_LIBRARIES} img)
SET_TARGET_PROPERTIES(img_manipulation PROPERTIES COMPILE_FLAGS "-fPIC" )
