cmake_minimum_required(VERSION 2.8) 
PROJECT (MyLineMod)
find_package(OpenCV REQUIRED )
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
FIND_PACKAGE( Boost 1.40 COMPONENTS system filesystem thread REQUIRED )
find_package(PCL 1.7 REQUIRED)
#find_package(ASSIMP REQUIRED)
#include_directories( /home/giorgio/linemod_plus_renderer/MyLinemodPlusRender/src/assimp/include )
find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ASSIMP assimp)
endif()
if(NOT ASSIMP_FOUND)
  message(WARNING "Assimp not found, not building synthetic views")
endif()
if(ASSIMP_FOUND AND NOT ${ASSIMP_VERSION} VERSION_LESS "3.0.0")
  message(STATUS "Found assimp v3")
  set(EXTRA_SOURCES)
  set(EXTRA_LIBRARIES assimp)
else()
  message(STATUS "Building assimp v3")
  set(ASSIMP_INCLUDE_DIRS src/assimp/include)
  aux_source_directory(src/assimp/contrib/clipper EXTRA_SOURCES_clipper)
  aux_source_directory(src/assimp/contrib/ConvertUTF EXTRA_SOURCES_ConvertUTF)
  aux_source_directory(src/assimp/contrib/irrXML EXTRA_SOURCES_irrXML)
  aux_source_directory(src/assimp/contrib/poly2tri/poly2tri/common EXTRA_SOURCES_poly2tri)
  aux_source_directory(src/assimp/contrib/poly2tri/poly2tri/sweep EXTRA_SOURCES_poly2tri_sweep)
  aux_source_directory(src/assimp/contrib/unzip EXTRA_SOURCES_unzip)
  aux_source_directory(src/assimp/contrib/zlib EXTRA_SOURCES_zlib)
  aux_source_directory(src/assimp/code EXTRA_SOURCES)
  set(EXTRA_SOURCES ${EXTRA_SOURCES} ${EXTRA_SOURCES_clipper} ${EXTRA_SOURCES_ConvertUTF} ${EXTRA_SOURCES_irrXML} ${EXTRA_SOURCES_poly2tri} ${EXTRA_SOURCES_poly2tri_sweep} ${EXTRA_SOURCES_unzip} ${EXTRA_SOURCES_zlib})
  set(EXTRA_LIBRARIES)
endif()

include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} 
                     ${ASSIMP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}
                     ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
                     
message(STATUS ${OpenCV_VERSION})
message(STATUS ${OpenCV_INCLUDE_DIRS})
message(STATUS ${OpenCV_LIBS})

#set( NAME_SRC
#    src/*.cpp    
#)

#set( NAME_HEADERS       
#     include/*.h
#)

#file(GLOB LineModTrainigSRC 
#   src/*.cpp
#   src/*.h
#   ${EXTRA_SOURCES}
#   #src/assimp/include/*
#   #src/assimp/*
#)
file(GLOB LineModTrainigSRC 
#   src/linemod_train.cpp
   src/model.cpp
   src/renderer3d.cpp
   src/renderer3d_impl_glut.cpp
   src/utils.cpp
   
   src/model.h
   src/renderer.h
   src/renderer3d.h
   src/renderer3d_impl_glut.h
   src/renderer3d_impl_base.h
   src/utils.h
   ${EXTRA_SOURCES}
   #src/assimp/include/*
   #src/assimp/*
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#file(GLOB LineModDetectionSRC 
#   src/MyLinemod_detect.cpp
#   src/model.cpp
#   src/renderer3d.cpp
#   src/renderer3d_impl_glut.cpp
#   src/utils.cpp
#   src/linemod_icp.cpp
#   
#   src/model.h
#   src/renderer.h
#   src/renderer3d.h
#   src/renderer3d_impl_glut.h
#   src/renderer3d_impl_base.h
#   src/utils.h
#   src/db_linemod.h
#   src/linemod_icp.h
#   ${EXTRA_SOURCES}
#   #src/assimp/include/*
#   #src/assimp/*
#)

#INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
#link_directories( ${CMAKE_BINARY_DIR}/bin)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
add_executable( LineModTraining src/linemod_train.cpp ${LineModTrainigSRC}) #${NAME_SRC} ${NAME_HEADERS} )
target_link_libraries( LineModTraining ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${Boost_LIBRARIES} freeimage)

add_executable( LineModDetection  src/MyLinemod_detect.cpp src/linemod_icp.cpp ${LineModTrainigSRC})
target_link_libraries( LineModDetection ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${Boost_LIBRARIES} ${PCL_LIBRARIES} freeimage)
 #/opt/ros/hydro/lib/libopencv_rgbd.so

