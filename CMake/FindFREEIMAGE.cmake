FIND_PATH(FREEIMAGE_INCLUDE_DIRS NAMES "FreeImage.h" PATHS /opt/inc /include /usr/include /include/freeimage /usr/include/freeimage ${FREEIMAGE_INCLUDE_DIR} DOC "The FREEIMAGE include directory")
FIND_LIBRARY(FREEIMAGE_LIBRARIES NAMES "freeimage" "freeimageplus" PATHS /opt/lib /opt/freeimage/lib /opt/freeimage/lib /usr/lib/freeimage /lib/freeimage /opt/freeimage/ ${CMAKE_PREFIX_PATH}/freeimage ${FREEIMAGE_LIB_DIR} DOC "The FREEIMAGE library (libfreeimage.so)")
message("FREEIMAGE_INCLUDE_DIRS: ${FREEIMAGE_INCLUDE_DIRS}; FREEIMAGE_LIBRARIES: ${FREEIMAGE_LIBRARIES}")

SET(FREEIMAGE_FOUND FALSE)

IF(FREEIMAGE_INCLUDE_DIRS AND FREEIMAGE_LIBRARIES)
  SET(FREEIMAGE_FOUND TRUE)
ELSE()
  MESSAGE(WARNING "Couldn't find FREEIMAGE libraries. Have you set FREEIMAGE_INCLUDE_DIR and FREEIMAGE_LIB_DIR to the root of the leap lib? Try again with cmake -DFREEIMAGE_INCLUDE_DIR=/path/to/freeimage/include -DFREEIMAGE_LIB_DIR=/path/to/freeimage/lib :)")
  SET(FREEIMAGE_FOUND FALSE)
ENDIF()

#Marks as advanced variables the paths, so the gui-user will not modify it (but who cares? Who uses GUIs anymore nowadays?)
MARK_AS_ADVANCED(FREEIMAGE_INCLUDE_DIRS FREEIMAGE_LIBRARIES)
