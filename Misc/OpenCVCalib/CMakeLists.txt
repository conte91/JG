find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(opencv_calib calibration.cpp)
target_link_libraries(opencv_calib ${OpenCV_LIBRARIES})

COPY_TO_BIN(calib_xml_from_folder.sh)
