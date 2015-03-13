FIND_PATH(Leap_INCLUDE_DIRS NAMES "Leap.h" "LeapMath.h" PATHS ENV Leap_INCLUDE_DIR DOC "The Leap motion SDK include directory")
message("Leap_INCLUDE_DIRS: ${Leap_INCLUDE_DIRS}; Leap_LIBRARIES: ${Leap_LIBRARIES}")
message("PREFIX: ${CMAKE_PREFIX_PATH}")
FIND_LIBRARY(Leap_LIBRARIES NAMES Leap PATHS /usr/lib32/Leap /lib32/Leap /usr/lib/Leap /lib/Leap /opt/Leap/ ${CMAKE_PREFIX_PATH}/Leap ENV Leap_LIB_DIR DOC "The Leap motion library (libLeap.so)")

SET(Leap_FOUND FALSE)

IF(Leap_INCLUDE_DIRS AND Leap_LIBRARIES)
  SET(Leap_FOUND TRUE)
ELSE()
  MESSAGE(WARNING "Couldn't find Leap libraries. Have you set Leap_INCLUDE_DIR and Leap_LIB_DIR to the root of the leap lib? Try again with cmake -DLeap_INCLUDE_DIR=/path/to/leap/include -DLeap_LIB_DIR=/path/to/leap/lib :)")
  SET(Leap_FOUND FALSE)
ENDIF()

#Marks as advanced variables the paths, so the gui-user will not modify it (but who cares? Who uses GUIs anymore nowadays?)
MARK_AS_ADVANCED(Leap_INCLUDE_DIRS Leap_LIBRARIES)
