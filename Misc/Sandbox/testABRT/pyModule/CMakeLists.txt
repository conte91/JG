FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)
FIND_PACKAGE(Boost COMPONENTS python system)
IF(NOT ${Boost_FOUND})
  MESSAGE("Couldn't find Boost/python: going on searching for python3..\n")
  FIND_PACKAGE(Boost COMPONENTS python3 system REQUIRED)
ENDIF(NOT ${Boost_FOUND})

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

PYTHON_ADD_MODULE(pyAPC ../main.cpp mainModule.cpp)

TARGET_LINK_LIBRARIES(pyAPC ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} robotdata camera c5g c5g_misc apc_main) 
