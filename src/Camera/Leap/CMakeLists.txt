find_package(Leap REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Leap_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(leapcamera 
  LeapCamera.cpp
)

target_link_libraries(leapcamera ${Leap_LIBRARIES} ${OpenCV_LIBRARIES} camera)
