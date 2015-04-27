FIND_PATH(assimp_INCLUDE_DIRS NAMES "assimp/scene.h" PATHS /opt/inc /include /usr/include /include/assimp /usr/include/assimp ${assimp_INCLUDE_DIR} DOC "The assimp include directory")
message("assimp_INCLUDE_DIRS: ${assimp_INCLUDE_DIRS}; assimp_LIBRARIES: ${assimp_LIBRARIES}")
message("PREFIX: ${CMAKE_PREFIX_PATH}")
FIND_LIBRARY(assimp_LIBRARIES NAMES "assimp" PATHS /opt/lib /opt/assimp/lib /opt/assimp/lib /usr/lib/assimp /lib/assimp /opt/assimp/ /usr/lib /lib ${CMAKE_PREFIX_PATH}/assimp ${assimp_LIB_DIR} DOC "The assimp library (libassimp.so)")

SET(assimp_FOUND FALSE)

IF(assimp_INCLUDE_DIRS AND assimp_LIBRARIES)
  SET(assimp_FOUND TRUE)
ELSE()
  MESSAGE(WARNING "Couldn't find assimp libraries. Have you set assimp_INCLUDE_DIR and assimp_LIB_DIR to the root of the leap lib? Try again with cmake -Dassimp_INCLUDE_DIR=/path/to/assimp/include -Dassimp_LIB_DIR=/path/to/assimp/lib :)")
  SET(assimp_FOUND FALSE)
ENDIF()

#Marks as advanced variables the paths, so the gui-user will not modify it (but who cares? Who uses GUIs anymore nowadays?)
MARK_AS_ADVANCED(assimp_INCLUDE_DIRS assimp_LIBRARIES)
