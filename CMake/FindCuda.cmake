FIND_PATH(Cuda_INCLUDE_DIRS NAMES "cuda.h" PATHS /usr/include /usr/include/cuda /usr/share/cuda/include /opt/cuda/include ${Cuda_INCLUDE_DIR} DOC "The Cuda include directory")
FIND_LIBRARY(Cuda_LIBRARIES NAMES "cudart" PATHS /opt/cuda/lib64 /opt/cuda/lib /usr/lib/cuda /lib/cuda /usr/lib /lib /usr/lib64 /lib64 /opt/cuda/ ${CMAKE_PREFIX_PATH}/cuda ${Cuda_LIB_DIR} DOC "The Cuda library (libCuda.so)")

message("Cuda_INCLUDE_DIRS: ${Cuda_INCLUDE_DIRS}; Cuda_LIBRARIES: ${Cuda_LIBRARIES}")

SET(Cuda_FOUND FALSE)

IF(Cuda_INCLUDE_DIRS AND Cuda_LIBRARIES)
  SET(Cuda_FOUND TRUE)
ELSE()
  MESSAGE(WARNING "Couldn't find Cuda libraries. Have you set Cuda_INCLUDE_DIR and Cuda_LIB_DIR to the root of the CUDA lib? Try again with cmake -DCuda_INCLUDE_DIR=/path/to/cuda/include -DCuda_LIB_DIR=/path/to/cuda/lib :)")
  SET(Cuda_FOUND FALSE)
ENDIF()

#Marks as advanced variables the paths, so the gui-user will not modify it (but who cares? Who uses GUIs anymore nowadays?)
MARK_AS_ADVANCED(Cuda_INCLUDE_DIRS Cuda_LIBRARIES)
