if(NOT(DEFINED NO_EORL))
  find_package(eORL REQUIRED)
ENDIF(NOT(DEFINED NO_EORL))

add_library(c5g_misc Pose.cpp)
