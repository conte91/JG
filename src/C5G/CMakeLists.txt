if(NOT(DEFINED NO_EORL))
  find_package(eORL REQUIRED)
  include_directories(${eORL_INCLUDE_DIRS})
  add_library(c5g C5G.cpp userCallback.c)
  target_link_libraries(c5g ${eORL_LIBRARIES})
ELSE(NOT(DEFINED NO_EORL))
  add_library(c5g C5G_dummy.cpp)
ENDIF(NOT(DEFINED NO_EORL))

add_library(c5g_misc Pose.cpp)
SET_TARGET_PROPERTIES(c5g_misc PROPERTIES COMPILE_FLAGS "-fPIC")

SET_TARGET_PROPERTIES(c5g PROPERTIES COMPILE_FLAGS "-fPIC")
TARGET_LINK_LIBRARIES(c5g c5g_misc)

