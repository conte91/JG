find_package(OpenCV REQUIRED)
if(NOT(DEFINED PCL_FOUND))
  find_package(PCL REQUIRED)
ENDIF(NOT(DEFINED PCL_FOUND))

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(calib_extr calibExtr.cpp)
target_link_libraries(calib_extr ${OpenCV_LIBRARIES} camera img)

add_executable(merge_calibration mergeCalibration.cpp)
target_link_libraries(merge_calibration ${OpenCV_LIBRARIES} camera)

add_executable(opencv_calib calibration.cpp)
target_link_libraries(opencv_calib ${OpenCV_LIBRARIES} camera)

COPY_TO_BIN(calib_xml_from_folder.sh)
