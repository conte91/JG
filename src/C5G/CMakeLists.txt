MESSAGE("Using ${ROBOT_TYPE} C5G class")

IF(NOT(DEFINED ROBOT_TYPE))
  MESSAGE(FATAL_ERROR "You shall define one ROBOT_TYPE to use")
ENDIF(NOT(DEFINED ROBOT_TYPE))

if((${ROBOT_TYPE} STREQUAL "EORL"))
  find_package(eORL REQUIRED)
  include_directories(${eORL_INCLUDE_DIRS})
  add_library(c5g SHARED C5G_eORL.cpp userCallback.c)
  target_link_libraries(c5g ${eORL_LIBRARIES})
ELSEIF((${ROBOT_TYPE} STREQUAL "SOCKET"))
  find_package(Boost COMPONENTS system thread REQUIRED)
  add_library(c5g SHARED C5G_socket.cpp )
  target_link_libraries(c5g ${Boost_LIBRARIES} pthread)
ELSEIF((${ROBOT_TYPE} STREQUAL "DUMMY"))
  add_library(c5g SHARED C5G_dummy.cpp)
ENDIF((${ROBOT_TYPE} STREQUAL "EORL"))

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})
add_library(c5g_misc SHARED Pose.cpp)
SET_TARGET_PROPERTIES(c5g_misc PROPERTIES COMPILE_FLAGS "-fPIC")

SET_TARGET_PROPERTIES(c5g PROPERTIES COMPILE_FLAGS "-fPIC")
TARGET_LINK_LIBRARIES(c5g c5g_misc)

