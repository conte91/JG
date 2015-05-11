find_package(Boost COMPONENTS system thread REQUIRED)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(setup_shelf_camera_position taratura.cpp)
target_link_libraries(setup_shelf_camera_position ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} camera c5g apc)

add_executable(setup_shelf_position taratura_posizioni.cpp)
target_link_libraries(setup_shelf_position c5g apc)
