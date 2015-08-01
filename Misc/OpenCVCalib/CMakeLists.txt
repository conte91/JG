find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

add_executable(merge_calibration mergeCalibration.cpp)
target_link_libraries(merge_calibration ${OpenCV_LIBRARIES} camera)

add_executable(opencv_calib calibration.cpp)
target_link_libraries(opencv_calib ${OpenCV_LIBRARIES} camera)

COPY_TO_BIN(calib_xml_from_folder.sh)
