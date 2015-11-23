PROJECT("Amazon Picking Challenge")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g3 -gdwarf-2")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -gdwarf-2")
ADD_DEFINITIONS("-std=c++11 -Wall -Wno-deprecated-declarations -Wno-reorder -pipe -g3")
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

MACRO(COPY_TO_LIB filename)
  MESSAGE("Copying ${filename} to ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${filename}")
  CONFIGURE_FILE("${filename}" "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${filename}" COPYONLY)
ENDMACRO(COPY_TO_LIB)

MACRO(COPY_TO_BIN filename)
  MESSAGE("Copying ${filename} to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${filename}")
  CONFIGURE_FILE("${filename}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${filename}" COPYONLY)
ENDMACRO(COPY_TO_BIN)

MACRO(MAYBE_FIND pkgname)
  IF(NOT(DEFINED ${pkgname}_FOUND))
    find_package(${pkgname} REQUIRED)
  ENDIF(NOT(DEFINED ${pkgname}_FOUND))
ENDMACRO(MAYBE_FIND)

include("Libraries.cmake")
include("ProjectList.cmake")
