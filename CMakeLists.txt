PROJECT("Amazon Picking Challenge")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(OPENCV_EXTRA_MODULES_PATH "./opencv_contrib/modules")

IF( NOT(DEFINED NO_EORL))
  IF( CMAKE_SIZEOF_VOID_P MATCHES 8 )
    MESSAGE("Sorry, only 32 bit architectures are supported :( Try again in a few years..")
    MESSAGE("I will continue the build excluding all the eORL-related stuff")
    SET(NO_EORL 1)
  ENDIF( CMAKE_SIZEOF_VOID_P MATCHES 8 )
ENDIF( NOT(DEFINED NO_EORL))

INCLUDE_DIRECTORIES("include")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/slib" )
include("ProjectList.cmake")

