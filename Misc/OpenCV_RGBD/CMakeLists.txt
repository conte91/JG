FIND_PACKAGE(OpenCV REQUIRED)

set(the_description "RGBD algorithms")
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${Cuda_INCLUDE_DIRS})
add_library(opencv_rgbd SHARED depth_to_3d.cpp utils.cpp)
target_link_libraries(opencv_rgbd opencv_core opencv_calib3d opencv_highgui opencv_imgproc )
