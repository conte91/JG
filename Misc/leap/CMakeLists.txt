IF(NOT(DEFINED NO_LEAP))
find_package(OpenCV REQUIRED)
find_package(Leap REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Leap_INCLUDE_DIRS})
add_executable(leap_show_img leaplistener.cpp main.cpp)

target_link_libraries(leap_show_img ${OpenCV_LIBRARIES} ${Leap_LIBRARIES})
ENDIF(NOT(DEFINED NO_LEAP))
