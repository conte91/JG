FIND_PATH(eORL_INCLUDE_DIRS NAMES "eORL.h" PATHS ENV eORL_INCLUDE_DIR DOC "The eORL include directory")
message("eORL_INCLUDE_DIRS: ${eORL_INCLUDE_DIRS}; eORL_LIBRARIES: ${eORL_LIBRARIES}")
message("PREFIX: ${CMAKE_PREFIX_PATH}")
FIND_LIBRARY(eORL_LIBRARIES NAMES "eORL" PATHS /opt/eorl/lib /opt/eORL/lib /usr/lib/eORL /lib/eORL /opt/eORL/ ${CMAKE_PREFIX_PATH}/eORL ENV eORL_LIB_DIR DOC "The eORL library (libeORL.so)")

SET(eORL_FOUND FALSE)

IF(eORL_INCLUDE_DIRS AND eORL_LIBRARIES)
  SET(eORL_FOUND TRUE)
ELSE()
  MESSAGE(WARNING "Couldn't find eORL libraries. Have you set eORL_INCLUDE_DIR and eORL_LIB_DIR to the root of the leap lib? Try again with cmake -DeORL_INCLUDE_DIR=/path/to/eorl/include -DeORL_LIB_DIR=/path/to/eorl/lib :)")
  SET(eORL_FOUND FALSE)
ENDIF()

#Marks as advanced variables the paths, so the gui-user will not modify it (but who cares? Who uses GUIs anymore nowadays?)
MARK_AS_ADVANCED(eORL_INCLUDE_DIRS eORL_LIBRARIES)
