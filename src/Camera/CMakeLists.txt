find_package(Leap REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Leap_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(camera 
  LeapCamera.cpp
  )

target_link_libraries(camera ${Leap_LIBRARIES} ${OpenCV_LIBRARIES})
